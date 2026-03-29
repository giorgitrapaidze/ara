"use strict";

const Anthropic = require("@anthropic-ai/sdk");

const client = new Anthropic.default();

// claude-sonnet-4-0 = claude-sonnet-4-20250514 (as requested)
const MODEL = "claude-sonnet-4-0";

function buildPrompt(sectors) {
  const { reference: refSectors, current: curSectors, lapDelta } = sectors;

  const sectorLines = refSectors.map((ref, i) => {
    const cur = curSectors[i];
    const speedDelta  = +(cur.speed.min  - ref.speed.min).toFixed(1);
    const brakeDelta  = +(cur.brake.peak - ref.brake.peak).toFixed(3);
    const timeDelta   = +(cur.t_end      - ref.t_end).toFixed(3);
    const throttleAppDelta = (cur.throttle.app_point_dist != null && ref.throttle.app_point_dist != null)
      ? +(cur.throttle.app_point_dist - ref.throttle.app_point_dist).toFixed(0)
      : null;

    return `
Sector ${i + 1} (dist ${ref.dist_start}–${ref.dist_end}m):
  Speed  → ref min ${ref.speed.min} km/h | cur min ${cur.speed.min} km/h (Δ ${speedDelta > 0 ? "+" : ""}${speedDelta} km/h)
           ref max ${ref.speed.max} km/h | cur max ${cur.speed.max} km/h
  Brake  → ref peak ${ref.brake.peak} | cur peak ${cur.brake.peak} (Δ ${brakeDelta > 0 ? "+" : ""}${brakeDelta})
           braking duration ref ${ref.brake.duration}s | cur ${cur.brake.duration}s
  Throttle on → ref dist ${ref.throttle.app_point_dist}m | cur dist ${cur.throttle.app_point_dist}m${throttleAppDelta != null ? ` (Δ ${throttleAppDelta > 0 ? "+" : ""}${throttleAppDelta}m)` : ""}
  Brake disc temp (front) → ref avg ${ref.brakes_disc.temp_avg_front}°C | cur avg ${cur.brakes_disc.temp_avg_front}°C
  Tyre temp avg → ref ${ref.tyres.temp_avg}°C | cur ${cur.tyres.temp_avg}°C
  RPM max → ref ${ref.engine.rpm_max} | cur ${cur.engine.rpm_max}
  Sector time → ref end ${ref.t_end}s | cur end ${cur.t_end}s (cumulative Δ ${timeDelta > 0 ? "+" : ""}${timeDelta}s)`;
  }).join("\n");

  return `You are Co-Driver, an elite AI racing coach analyzing telemetry from an Indy-style racing car.

## Lap Overview
- Total lap delta: current is ${lapDelta > 0 ? "+" : ""}${lapDelta.toFixed(3)}s vs reference
- The reference lap is a clean, optimal lap by an experienced driver
- The current lap has deliberate mistakes: late braking, early throttle application, and understeer

## Sector-by-Sector Telemetry (5 sectors)
${sectorLines}

## Your Task
For EACH of the 5 sectors, provide:
1. A diagnosis of what the driver did wrong (or right) compared to the reference
2. One concrete, actionable coaching tip they can apply immediately
3. Estimated time that can be recovered with the correction

Be specific with numbers from the telemetry. Use racing terminology correctly.
Focus on the most impactful improvements first.
Return your response as a JSON object with this structure:
{
  "overall_assessment": "string",
  "lap_delta_seconds": number,
  "sectors": [
    {
      "sector": 1,
      "diagnosis": "string",
      "coaching_tip": "string",
      "estimated_gain_seconds": number,
      "severity": "minor|moderate|major"
    }
  ],
  "priority_focus": "string (the single most important thing to fix)"
}`;
}

async function getCoachingTips(sectors) {
  const lapDelta = sectors.current[sectors.current.length - 1].t_end
                 - sectors.reference[sectors.reference.length - 1].t_end;

  const prompt = buildPrompt({
    reference: sectors.reference,
    current:   sectors.current,
    lapDelta,
  });

  const stream = client.messages.stream({
    model:      MODEL,
    max_tokens: 2048,
    thinking:   { type: "adaptive" },
    messages:   [{ role: "user", content: prompt }],
    system:     "You are Co-Driver, an elite AI racing coach. Always respond with valid JSON.",
  });

  const message = await stream.finalMessage();

  // Extract JSON from the text response
  const textBlock = message.content.find(b => b.type === "text");
  if (!textBlock) throw new Error("No text in Claude response");

  const raw = textBlock.text.trim();
  const jsonMatch = raw.match(/\{[\s\S]*\}/);
  if (!jsonMatch) throw new Error("No JSON found in Claude response");

  return {
    coaching: JSON.parse(jsonMatch[0]),
    model:    message.model,
    usage:    message.usage,
  };
}

module.exports = { getCoachingTips };
