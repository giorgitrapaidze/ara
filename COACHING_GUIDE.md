# Personal AI Racing Coach — Builder's Guide

## What You Have

Three MCAP files with rich multi-sensor telemetry from an autonomous car at Yas Marina:

| File | Use For |
|------|---------|
| `hackathon_good_lap.mcap` | Baseline / conservative reference |
| `hackathon_fast_laps.mcap` | Optimal reference lap (coaching target) |
| `hackathon_wheel_to_wheel.mcap` | Overtaking, race craft, defensive driving |

Plus `yas_marina_bnd.json` for track layout context.

---

## Key Data Signals for Coaching

| Signal | Coaching Use |
|--------|-------------|
| **Position (x, y, z) + yaw** | Racing line analysis — is the driver hitting apexes? |
| **Velocity (vx, vy)** | Speed comparison at each track sector |
| **Gas/brake/clutch pedals** | Braking points, trail braking, throttle application |
| **Slip ratios + slip angles** | Tyre usage, understeer/oversteer detection |
| **Steering angle** | Smoothness, entry/exit technique |
| **Wheel loads + suspension** | Load transfer, kerb riding |
| **Tyre temp/pressure** | Thermal management coaching |
| **Brake disc temps** | Brake management feedback |
| **RPM + gear** | Shift timing, engine utilization |

---

## Coaching App Architecture

```
MCAP Files → Parse telemetry → Segment by track sector
                                        ↓
                           Compare lap vs. fast reference
                                        ↓
                     Detect deltas (braking late? slow apex?)
                                        ↓
                        Claude API → Natural language feedback
                                        ↓
                              Racer dashboard / voice coach
```

---

## Core Features to Build

### 1. Lap Comparison
Diff a racer's lap against `hackathon_fast_laps.mcap` to find time loss per sector. Key signals: velocity, position, pedal inputs.

### 2. Braking Coach
Detect late/early braking points and missed trail-braking opportunities. Key signals: brake pressure, slip ratio, velocity at braking zone entry.

### 3. Racing Line Analysis
Compare the GPS trace to the optimal line using `yas_marina_bnd.json` track boundaries. Key signals: position (x, y), yaw, lateral offset from ideal line.

### 4. Tyre Coach
Flag overheating, lock-ups (slip ratio spikes), and front/rear balance issues. Key signals: tyre surface temps, tyre pressure, slip ratios, wheel loads.

### 5. Race Craft Coach
Use `hackathon_wheel_to_wheel.mcap` to analyze overtaking decisions, defensive positioning, and gap management. Key signals: position relative to opponents, steering, velocity delta.

### 6. Verbal Debrief
Feed structured sector deltas to the Claude API to generate a spoken or written coaching debrief — personalised, actionable, and in plain language.

---

## Quickstart Steps

1. **Install dependencies**
   ```bash
   pip install mcap mcap-ros2-support anthropic
   ```

2. **Parse MCAP** using the `mcap` Python library with the `sd_msgs` message definitions included in the repo.

3. **Extract primary signal** — `/constructor0/state_estimation` at ~100 Hz is the richest single source for coaching (position, velocity, pedals, slip, steering, gear, RPM).

4. **Segment laps** by track position using the circuit boundaries in `yas_marina_bnd.json`.

5. **Compare** racer lap vs. fast reference lap sector-by-sector to compute time and behaviour deltas.

6. **Generate feedback** by sending the structured deltas to Claude:

   ```python
   import anthropic

   client = anthropic.Anthropic()

   coaching_prompt = f"""
   You are an expert racing coach. Here is the sector-by-sector analysis
   of the driver's lap compared to the fastest reference lap at Yas Marina.

   {sector_deltas}

   Provide specific, actionable coaching feedback for each sector where
   the driver lost significant time. Be concise and focus on what to change.
   """

   message = client.messages.create(
       model="claude-opus-4-6",
       max_tokens=1024,
       messages=[{"role": "user", "content": coaching_prompt}]
   )
   print(message.content[0].text)
   ```

7. **Display or speak** the coaching output in a racer dashboard or via text-to-speech.

---

## Suggested Data Flow per Lap

```
state_estimation (100 Hz)
  └─ position → sector segmentation → line delta vs. reference
  └─ velocity → speed trace → min/max speed per corner
  └─ gas/brake → input timing → braking point delta
  └─ slip_ratio → lock-up / wheelspin detection
  └─ steering_angle → smoothness score

badenia_560_tpms + tyre_surface_temp
  └─ tyre temps → overheating / cold tyre warnings
  └─ tyre pressure → pressure loss detection

ice_status_01/02
  └─ RPM + gear → shift point analysis
```

---

## Files Reference

| File | Description |
|------|-------------|
| `hackathon_good_lap.mcap` | 81.3 s conservative lap — good as a novice baseline |
| `hackathon_fast_laps.mcap` | 74.3 s — two fastest laps, primary coaching reference |
| `hackathon_wheel_to_wheel.mcap` | 226 s multi-car race — race craft analysis |
| `yas_marina_bnd.json` | Circuit boundary geometry for line analysis |
| `sd_msgs/` | ROS 2 message definitions for deserialising MCAP topics |
