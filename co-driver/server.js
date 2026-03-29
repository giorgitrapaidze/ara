"use strict";

const express  = require("express");
const cors     = require("cors");
const path     = require("path");
const fs       = require("fs");

const { buildLaps }       = require("./src/simulator");
const { getCoachingTips } = require("./src/coaching");

const app    = express();
const PORT   = 4000;
const OUTDIR = __dirname;

// ── Middleware ───────────────────────────────────────────────────────────────

app.use(cors({ origin: ["http://localhost:5173", "http://127.0.0.1:5173"] }));
app.use(express.json());

// ── In-memory data store (built once at startup) ──────────────────────────────

let data = null;   // populated by init()

async function init() {
  console.log("Co-Driver starting up — simulating laps…");
  data = await buildLaps(OUTDIR);
  console.log("Ready. Listening on http://localhost:" + PORT);
}

// ── Routes ───────────────────────────────────────────────────────────────────

/**
 * GET /api/laps
 * Returns full 60 Hz telemetry for both laps as JSON.
 * ~5-6 MB response — use /api/download/:lap for Foxglove.
 */
app.get("/api/laps", (req, res) => {
  if (!data) return res.status(503).json({ error: "Initialising — try again in a moment" });

  res.json({
    meta: {
      hz:           60,
      track_length_m: data.trackLength,
      reference: {
        lap_time_s: +data.reference.lapTime.toFixed(3),
        frames:     data.reference.frames.length,
      },
      current: {
        lap_time_s: +data.current.lapTime.toFixed(3),
        frames:     data.current.frames.length,
      },
      delta_s: +(data.current.lapTime - data.reference.lapTime).toFixed(3),
    },
    reference: data.reference.frames,
    current:   data.current.frames,
  });
});

/**
 * GET /api/sectors
 * Returns the lap divided into 5 sectors with ref vs current comparison.
 */
app.get("/api/sectors", (req, res) => {
  if (!data) return res.status(503).json({ error: "Initialising — try again in a moment" });

  const refSectors = data.sectors.reference;
  const curSectors = data.sectors.current;

  const comparison = refSectors.map((ref, i) => {
    const cur = curSectors[i];
    const timeDelta   = +(cur.t_end      - ref.t_end).toFixed(3);
    const speedDelta  = +(cur.speed.min  - ref.speed.min).toFixed(1);
    const speedDeltaMax = +(cur.speed.max - ref.speed.max).toFixed(1);
    const brakesDelta = +(cur.brake.peak - ref.brake.peak).toFixed(3);
    const throttleAppDelta = (cur.throttle.app_point_dist != null && ref.throttle.app_point_dist != null)
      ? cur.throttle.app_point_dist - ref.throttle.app_point_dist
      : null;

    return {
      sector:     i + 1,
      dist_range: { start: ref.dist_start, end: ref.dist_end },
      reference:  ref,
      current:    cur,
      delta: {
        time_s:             timeDelta,
        speed_min_kmh:      speedDelta,
        speed_max_kmh:      speedDeltaMax,
        brake_peak:         brakesDelta,
        throttle_app_dist_m: throttleAppDelta,
        tyre_temp_avg_c:    +(cur.tyres.temp_avg - ref.tyres.temp_avg).toFixed(1),
        disc_temp_front_c:  +(cur.brakes_disc.temp_avg_front - ref.brakes_disc.temp_avg_front).toFixed(0),
      },
    };
  });

  res.json({
    lap_delta_s: +(data.current.lapTime - data.reference.lapTime).toFixed(3),
    sectors:     comparison,
  });
});

/**
 * GET /api/coaching
 * Calls the Anthropic API and returns AI coaching tips per sector.
 * Requires ANTHROPIC_API_KEY environment variable.
 */
app.get("/api/coaching", async (req, res) => {
  if (!data) return res.status(503).json({ error: "Initialising — try again in a moment" });

  try {
    const result = await getCoachingTips(data.sectors);
    res.json(result);
  } catch (err) {
    console.error("Coaching error:", err.message);

    if (err.message.includes("API key")) {
      return res.status(401).json({ error: "Invalid or missing ANTHROPIC_API_KEY" });
    }
    res.status(500).json({ error: err.message });
  }
});

/**
 * GET /api/summary
 * Returns overall delta table and top 3 improvements ranked by time gain.
 */
app.get("/api/summary", (req, res) => {
  if (!data) return res.status(503).json({ error: "Initialising — try again in a moment" });

  const refSectors = data.sectors.reference;
  const curSectors = data.sectors.current;

  // Per-sector delta table
  const sectorTable = refSectors.map((ref, i) => {
    const cur = curSectors[i];
    return {
      sector:           i + 1,
      dist_range:       `${ref.dist_start}–${ref.dist_end}m`,
      time_delta_s:     +(cur.t_end      - ref.t_end).toFixed(3),
      speed_min_delta:  +(cur.speed.min  - ref.speed.min).toFixed(1),
      speed_max_delta:  +(cur.speed.max  - ref.speed.max).toFixed(1),
      brake_peak_delta: +(cur.brake.peak - ref.brake.peak).toFixed(3),
      throttle_app_delta_m: (cur.throttle.app_point_dist != null && ref.throttle.app_point_dist != null)
        ? cur.throttle.app_point_dist - ref.throttle.app_point_dist
        : null,
      disc_temp_delta_c:  +(cur.brakes_disc.temp_avg_front - ref.brakes_disc.temp_avg_front).toFixed(0),
      tyre_temp_delta_c:  +(cur.tyres.temp_avg - ref.tyres.temp_avg).toFixed(1),
    };
  });

  // Top 3 improvements (sectors with biggest positive time delta)
  const top3 = [...sectorTable]
    .sort((a, b) => b.time_delta_s - a.time_delta_s)
    .slice(0, 3)
    .map(s => ({
      sector:          s.sector,
      time_to_gain_s:  s.time_delta_s,
      dist_range:      s.dist_range,
      key_issue: buildKeyIssue(s),
    }));

  res.json({
    overall: {
      reference_lap_time_s: +data.reference.lapTime.toFixed(3),
      current_lap_time_s:   +data.current.lapTime.toFixed(3),
      total_delta_s:        +(data.current.lapTime - data.reference.lapTime).toFixed(3),
      track_length_m:       data.trackLength,
    },
    sector_table: sectorTable,
    top_3_improvements: top3,
  });
});

/**
 * GET /api/download/:lap
 * Serves the MCAP file for Foxglove Studio.
 * :lap = "reference" or "current"
 */
app.get("/api/download/:lap", (req, res) => {
  if (!data) return res.status(503).json({ error: "Initialising — try again in a moment" });

  const lap = req.params.lap;
  if (lap !== "reference" && lap !== "current") {
    return res.status(400).json({ error: "lap must be 'reference' or 'current'" });
  }

  const filePath = data[lap].mcap;
  if (!fs.existsSync(filePath)) {
    return res.status(404).json({ error: "MCAP file not found" });
  }

  res.download(filePath, `${lap}.mcap`);
});

// ── Health check ─────────────────────────────────────────────────────────────

app.get("/api/health", (req, res) => {
  res.json({
    status: data ? "ready" : "initialising",
    uptime: process.uptime(),
  });
});

// ── Helpers ──────────────────────────────────────────────────────────────────

function buildKeyIssue(s) {
  const issues = [];
  if (s.brake_peak_delta > 0.08)         issues.push("late braking (high peak brake pressure)");
  if (s.throttle_app_delta_m != null && s.throttle_app_delta_m > 15) issues.push("early throttle application");
  if (s.speed_min_delta < -8)            issues.push("excess understeer / low minimum speed");
  if (s.disc_temp_delta_c > 80)          issues.push("brake overheating (late braking zone)");
  return issues.length ? issues.join(", ") : "marginal losses across zone";
}

// ── Start ─────────────────────────────────────────────────────────────────────

app.listen(PORT, async () => {
  await init();
});
