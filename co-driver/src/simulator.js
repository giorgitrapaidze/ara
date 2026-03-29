"use strict";

const fs   = require("fs");
const path = require("path");
const { McapWriter } = require("@mcap/core");

// ── Constants ────────────────────────────────────────────────────────────────

const HZ          = 60;
const DT          = 1 / HZ;
const MAX_ACCEL   = 14;   // m/s²
const MAX_DECEL   = 38;   // m/s²
const WHEEL_CIRC  = 1.92; // metres
const FINAL_DRIVE = 3.8;
const GEAR_RATIOS = [0, 3.50, 2.48, 1.88, 1.50, 1.23, 1.04, 0.92];
const GEAR_UP     = [0, 9500, 9200, 9000, 8800, 8600, 8400, 9999];
const GEAR_DN     = [0,    0, 5500, 5200, 5000, 4800, 4600, 4400];
const TYRE_PSI    = { FL: 27.4, FR: 27.4, RL: 26.8, RR: 26.8 };

// ── Circuit definition ───────────────────────────────────────────────────────
// curv in rad/m, len in metres

const SEGMENTS = [
  { id:  0, name: "S/F Straight",   curv:  0,      len: 380 },
  { id:  1, name: "T1 Hairpin",     curv:  0.024,  len: 185 },
  { id:  2, name: "T1-T2 Link",     curv:  0,      len:  85 },
  { id:  3, name: "T2 Chicane-L",   curv: -0.022,  len: 105 },
  { id:  4, name: "T3 Chicane-R",   curv:  0.022,  len: 105 },
  { id:  5, name: "Back Straight",  curv:  0,      len: 375 },
  { id:  6, name: "T5 Braking",     curv:  0.021,  len: 155 },
  { id:  7, name: "T6 Sweeper",     curv: -0.012,  len: 215 },
  { id:  8, name: "T7 Complex-L",   curv:  0.018,  len: 125 },
  { id:  9, name: "T8 Complex-R",   curv: -0.016,  len: 125 },
  { id: 10, name: "Inner Straight", curv:  0,      len: 145 },
  { id: 11, name: "T10 Hairpin",    curv:  0.027,  len: 135 },
  { id: 12, name: "T11 Exit",       curv: -0.011,  len: 175 },
  { id: 13, name: "Pit Straight",   curv:  0,      len: 285 },
];

// Total track length in metres (1 point per metre)
const TRACK_LEN = SEGMENTS.reduce((s, g) => s + Math.round(g.len), 0);

// Corner speed target based on absolute curvature
function vTarget(absCurv) {
  if (absCurv === 0)      return 285;
  if (absCurv <= 0.012)   return 195;
  if (absCurv <= 0.018)   return 160;
  if (absCurv <= 0.022)   return 125;
  return 92;
}

// ── Track geometry ───────────────────────────────────────────────────────────

function buildTrack() {
  const pts = [];
  let x = 0, y = 0, yaw = 0, dist = 0;
  for (const seg of SEGMENTS) {
    const n = Math.round(seg.len);
    for (let i = 0; i < n; i++) {
      pts.push({
        x, y, yaw,
        curv:    seg.curv,
        vTarget: vTarget(Math.abs(seg.curv)),
        dist,
        seg:     seg.name,
        segId:   seg.id,
      });
      yaw  += seg.curv;
      x    += Math.cos(yaw);
      y    += Math.sin(yaw);
      dist += 1;
    }
  }
  return pts;
}

// ── Speed profile ────────────────────────────────────────────────────────────

function computeSpeedProfile(pts, imp = {}) {
  const n = pts.length;
  const v = Float64Array.from(pts, p => p.vTarget);

  // Late braking at T1 (id 1) and T5 (id 6):
  //   - LOWER the apex speed (driver runs wide, missed apex = less grip)
  //   - Raise approach speed 50 m before corner (driver hasn't braked yet)
  //   Net: harder braking over shorter distance, slower corner minimum
  if (imp.lateBraking) {
    const APEX_SEGS = new Set([1, 6]);
    for (let i = 0; i < n; i++) {
      if (APEX_SEGS.has(pts[i].segId)) {
        v[i] = Math.max(v[i] * (1 - imp.lateBraking * 0.13), 65);
      }
    }
    // Raise approach speed 50 m before each problem corner
    for (let i = 0; i < n; i++) {
      const ahead50 = pts[(i + 50) % n].segId;
      if (APEX_SEGS.has(ahead50) && !APEX_SEGS.has(pts[i].segId)) {
        v[i] = Math.min(v[i] * (1 + imp.lateBraking * 0.10), 290);
      }
    }
  }

  // Early throttle at chicane (id 3, 4):
  //   - Driver stabs throttle mid-corner → car pushes wide → must lift off
  //   - Result: lower corner speed and messy exit
  if (imp.earlyThrottle) {
    for (let i = 0; i < n; i++) {
      if (pts[i].segId === 3 || pts[i].segId === 4) {
        v[i] = Math.max(v[i] * (1 - imp.earlyThrottle * 0.09), 65);
      }
    }
  }

  // Understeer at T6 sweeper (id 7):
  //   - Driver can't rotate car → takes wider line → lower sustainable speed
  if (imp.understeer) {
    for (let i = 0; i < n; i++) {
      if (pts[i].segId === 7) {
        v[i] = Math.max(v[i] * (1 - imp.understeer * 0.12), 120);
      }
    }
  }

  // Backward pass — braking limits (3 iterations for convergence)
  for (let pass = 0; pass < 3; pass++) {
    for (let i = n - 1; i >= 0; i--) {
      const j  = (i + 1) % n;
      const vj = v[j] / 3.6;
      v[i] = Math.min(v[i], Math.sqrt(vj * vj + 2 * MAX_DECEL) * 3.6);
    }
  }

  // Forward pass — acceleration limits
  for (let pass = 0; pass < 3; pass++) {
    for (let i = 0; i < n; i++) {
      const j  = (i + 1) % n;
      const vi = v[i] / 3.6;
      v[j] = Math.min(v[j], Math.sqrt(vi * vi + 2 * MAX_ACCEL) * 3.6);
    }
  }

  return v;
}

// ── Physics helpers ──────────────────────────────────────────────────────────

function lerp(a, b, t) { return a + (b - a) * t; }
function n1(x)         { return +(Math.random() - 0.5) * x; }

function speedToRpm(speedKmh, g) {
  const speedMs = speedKmh / 3.6;
  return (speedMs / WHEEL_CIRC) * GEAR_RATIOS[g] * FINAL_DRIVE * 60;
}

// ── Core simulation ──────────────────────────────────────────────────────────

function simulateLap(pts, speedProfile) {
  const frames = [];
  const n = pts.length;

  let dist  = 0;
  let t     = 0;
  let gear  = 2;
  const ty  = { FL: 72, FR: 72, RL: 78, RR: 78 };   // tyre temps °C
  const bt  = { FL: 150, FR: 150, RL: 85, RR: 85 };  // brake disc temps °C

  while (dist < n - 1) {
    const i = Math.floor(dist) % n;
    const f = dist - Math.floor(dist);
    const j = (i + 1) % n;

    const speed = lerp(speedProfile[i], speedProfile[j], f);
    const x     = lerp(pts[i].x,   pts[j].x,   f);
    const y     = lerp(pts[i].y,   pts[j].y,   f);
    const yaw   = lerp(pts[i].yaw, pts[j].yaw,  f);
    const curv  = pts[i].curv;

    const speedMs = speed / 3.6;
    const vx = speedMs * Math.cos(yaw);
    const vy = speedMs * Math.sin(yaw);

    // Throttle / brake from 5-step look-ahead
    const vAhead = speedProfile[(i + 5) % n];
    let throttle, brake;
    if (vAhead > speed + 1.5) {
      throttle = Math.min(1, (vAhead - speed) / 40);
      brake    = 0;
    } else if (vAhead < speed - 2) {
      throttle = 0;
      brake    = Math.min(1, (speed - vAhead) / 80);
    } else {
      throttle = Math.max(0.05, 0.28 - Math.abs(curv) * 8);
      brake    = 0;
    }
    const steering = Math.max(-1, Math.min(1, curv * 46 + n1(0.015)));

    // Gear & RPM
    let rpm = speedToRpm(speed, gear);
    if (gear < 7 && rpm > GEAR_UP[gear]) gear++;
    if (gear > 1 && rpm < GEAR_DN[gear]) gear--;
    rpm = Math.max(4000, Math.min(12000, speedToRpm(speed, gear) + n1(120)));

    // G-forces
    const latG = curv * speedMs * speedMs / 9.81;
    const lonG = (throttle * MAX_ACCEL - brake * MAX_DECEL) / 9.81;

    // Tyre temperatures
    for (const c of ["FL", "FR", "RL", "RR"]) {
      const loaded = (latG > 0 && (c === "FR" || c === "RR")) ||
                     (latG < 0 && (c === "FL" || c === "RL"));
      const heat = (Math.abs(latG) * 1.6 + brake * 4.5 + throttle * 1.3) *
                   (loaded ? 1.4 : 0.65);
      ty[c] = Math.max(65, Math.min(118, ty[c] + (heat - 1.3) * DT));
    }

    // Brake disc temperatures
    for (const c of ["FL", "FR", "RL", "RR"]) {
      const front = c === "FL" || c === "FR";
      bt[c] = Math.max(100, Math.min(950,
        bt[c] + (brake * (front ? 920 : 420) - 65) * DT));
    }

    // Slip ratios
    const slipRatio = {
      FL: +(throttle * 0.055 + brake * 0.145 + n1(0.008)).toFixed(4),
      FR: +(throttle * 0.055 + brake * 0.145 + n1(0.008)).toFixed(4),
      RL: +(throttle * 0.180 + brake * 0.070 + n1(0.008)).toFixed(4),
      RR: +(throttle * 0.180 + brake * 0.070 + n1(0.008)).toFixed(4),
    };

    // Slip angles (degrees)
    const saF = Math.atan(latG * 0.42) * (180 / Math.PI);
    const saR = Math.atan(latG * 0.30) * (180 / Math.PI);
    const slipAngle = {
      FL: +(saF + n1(0.25)).toFixed(3),
      FR: +(saF + n1(0.25)).toFixed(3),
      RL: +(saR + n1(0.25)).toFixed(3),
      RR: +(saR + n1(0.25)).toFixed(3),
    };

    // Wheel loads (N) — 1400 kg car, ~3500 N nominal per corner
    const BASE_LOAD = 3500;
    const dLat = latG * 420;
    const dLon = lonG * 290;
    const load = {
      FL: Math.round(Math.max(120, BASE_LOAD - dLat + dLon)),
      FR: Math.round(Math.max(120, BASE_LOAD + dLat + dLon)),
      RL: Math.round(Math.max(120, BASE_LOAD - dLat - dLon)),
      RR: Math.round(Math.max(120, BASE_LOAD + dLat - dLon)),
    };

    // Suspension travel (metres)
    const K = 22000; // N/m spring rate
    const susTravel = {
      FL: +((load.FL - BASE_LOAD) / K).toFixed(5),
      FR: +((load.FR - BASE_LOAD) / K).toFixed(5),
      RL: +((load.RL - BASE_LOAD) / K).toFixed(5),
      RR: +((load.RR - BASE_LOAD) / K).toFixed(5),
    };

    frames.push({
      t:     +t.toFixed(4),
      dist:  +dist.toFixed(1),
      pose:       { x: +x.toFixed(2), y: +y.toFixed(2), z: 0, yaw: +yaw.toFixed(4) },
      velocity:   { vx: +vx.toFixed(3), vy: +vy.toFixed(3), speed: +speed.toFixed(2) },
      inputs:     { throttle: +throttle.toFixed(3), brake: +brake.toFixed(3), clutch: 0, steering: +steering.toFixed(4) },
      tyres:      {
        slip_ratio: { ...slipRatio },
        slip_angle: { ...slipAngle },
        temp:       { FL: +ty.FL.toFixed(1), FR: +ty.FR.toFixed(1), RL: +ty.RL.toFixed(1), RR: +ty.RR.toFixed(1) },
        pressure:   { ...TYRE_PSI },
      },
      suspension: { load: { ...load }, travel: { ...susTravel } },
      engine:     { rpm: Math.round(rpm), gear },
      brakes:     { disc_temp: { FL: Math.round(bt.FL), FR: Math.round(bt.FR), RL: Math.round(bt.RL), RR: Math.round(bt.RR) } },
    });

    dist += speedMs * DT;
    t    += DT;
  }

  return frames;
}

// ── MCAP writer ──────────────────────────────────────────────────────────────

class FileWritable {
  constructor(filePath) {
    this.fd   = fs.openSync(filePath, "w");
    this._pos = 0n;
  }
  async write(buf) {
    fs.writeSync(this.fd, buf);
    this._pos += BigInt(buf.byteLength);
  }
  position() { return this._pos; }
  close()    { fs.closeSync(this.fd); }
}

function schemaFor(name, props) {
  return {
    name,
    encoding: "jsonschema",
    data: Buffer.from(JSON.stringify({ type: "object", properties: props }), "utf8"),
  };
}

async function writeMcap(frames, filePath) {
  const fw     = new FileWritable(filePath);
  const writer = new McapWriter({ writable: fw });
  await writer.start({ library: "co-driver", profile: "" });

  // Schema definitions
  const schemas = {
    pose: await writer.registerSchema(schemaFor("car/Pose",
      { x: {type:"number"}, y: {type:"number"}, z: {type:"number"}, yaw: {type:"number"} })),
    velocity: await writer.registerSchema(schemaFor("car/Velocity",
      { vx: {type:"number"}, vy: {type:"number"}, speed: {type:"number"} })),
    inputs: await writer.registerSchema(schemaFor("car/Inputs",
      { throttle: {type:"number"}, brake: {type:"number"}, clutch: {type:"number"}, steering: {type:"number"} })),
    tyres: await writer.registerSchema(schemaFor("car/Tyres",
      { slip_ratio: {type:"object"}, slip_angle: {type:"object"}, temp: {type:"object"}, pressure: {type:"object"} })),
    suspension: await writer.registerSchema(schemaFor("car/Suspension",
      { load: {type:"object"}, travel: {type:"object"} })),
    engine: await writer.registerSchema(schemaFor("car/Engine",
      { rpm: {type:"number"}, gear: {type:"number"} })),
    brakes: await writer.registerSchema(schemaFor("car/Brakes",
      { disc_temp: {type:"object"} })),
  };

  const channels = {
    pose:       await writer.registerChannel({ topic: "/car/pose",       messageEncoding: "json", schemaId: schemas.pose,       metadata: new Map() }),
    velocity:   await writer.registerChannel({ topic: "/car/velocity",   messageEncoding: "json", schemaId: schemas.velocity,   metadata: new Map() }),
    inputs:     await writer.registerChannel({ topic: "/car/inputs",     messageEncoding: "json", schemaId: schemas.inputs,     metadata: new Map() }),
    tyres:      await writer.registerChannel({ topic: "/car/tyres",      messageEncoding: "json", schemaId: schemas.tyres,      metadata: new Map() }),
    suspension: await writer.registerChannel({ topic: "/car/suspension", messageEncoding: "json", schemaId: schemas.suspension, metadata: new Map() }),
    engine:     await writer.registerChannel({ topic: "/car/engine",     messageEncoding: "json", schemaId: schemas.engine,     metadata: new Map() }),
    brakes:     await writer.registerChannel({ topic: "/car/brakes",     messageEncoding: "json", schemaId: schemas.brakes,     metadata: new Map() }),
  };

  let seq = 0;
  for (const frame of frames) {
    const ns = BigInt(Math.round(frame.t * 1e9));
    const msgs = {
      pose:       frame.pose,
      velocity:   frame.velocity,
      inputs:     frame.inputs,
      tyres:      frame.tyres,
      suspension: frame.suspension,
      engine:     frame.engine,
      brakes:     frame.brakes,
    };
    for (const [key, channelId] of Object.entries(channels)) {
      await writer.addMessage({
        channelId,
        sequence:    seq++,
        logTime:     ns,
        publishTime: ns,
        data:        Buffer.from(JSON.stringify(msgs[key]), "utf8"),
      });
    }
  }

  await writer.end();
  fw.close();
}

// ── Sector helpers ───────────────────────────────────────────────────────────

function computeSectors(frames, n = 5) {
  const chunkSize = Math.ceil(frames.length / n);
  return Array.from({ length: n }, (_, i) => {
    const slice = frames.slice(i * chunkSize, (i + 1) * chunkSize);
    if (!slice.length) return null;

    const speeds    = slice.map(f => f.velocity.speed);
    const brakes    = slice.map(f => f.inputs.brake);
    const throttles = slice.map(f => f.inputs.throttle);
    const latGs     = slice.map(f => f.pose.yaw);  // proxy via yaw-change
    const avg       = arr => arr.reduce((a, b) => a + b, 0) / arr.length;
    const max       = arr => Math.max(...arr);
    const min       = arr => Math.min(...arr);

    // Find throttle application point (first frame where throttle > 0.5)
    const throttleOnIdx = slice.findIndex(f => f.inputs.throttle > 0.5);

    return {
      sector: i + 1,
      frames: slice.length,
      t_start: +slice[0].t.toFixed(3),
      t_end:   +slice[slice.length - 1].t.toFixed(3),
      dist_start: +slice[0].dist.toFixed(0),
      dist_end:   +slice[slice.length - 1].dist.toFixed(0),
      speed: {
        min: +min(speeds).toFixed(1),
        max: +max(speeds).toFixed(1),
        avg: +avg(speeds).toFixed(1),
      },
      brake: {
        peak:     +max(brakes).toFixed(3),
        avg:      +avg(brakes).toFixed(3),
        duration: +(slice.filter(f => f.inputs.brake > 0.05).length / HZ).toFixed(2),
      },
      throttle: {
        peak:         +max(throttles).toFixed(3),
        avg:          +avg(throttles).toFixed(3),
        app_point_t:  throttleOnIdx >= 0 ? +slice[throttleOnIdx].t.toFixed(3) : null,
        app_point_dist: throttleOnIdx >= 0 ? +slice[throttleOnIdx].dist.toFixed(0) : null,
      },
      tyres: {
        temp_avg: +(avg(slice.map(f => (f.tyres.temp.FL + f.tyres.temp.FR + f.tyres.temp.RL + f.tyres.temp.RR) / 4))).toFixed(1),
        temp_max: +(max(slice.map(f => Math.max(f.tyres.temp.FL, f.tyres.temp.FR, f.tyres.temp.RL, f.tyres.temp.RR)))).toFixed(1),
      },
      brakes_disc: {
        temp_max_front: +(max(slice.map(f => Math.max(f.brakes.disc_temp.FL, f.brakes.disc_temp.FR)))).toFixed(0),
        temp_avg_front: +(avg(slice.map(f => (f.brakes.disc_temp.FL + f.brakes.disc_temp.FR) / 2))).toFixed(0),
      },
      engine: {
        rpm_max: Math.round(max(slice.map(f => f.engine.rpm))),
        gear_max: Math.round(max(slice.map(f => f.engine.gear))),
      },
    };
  }).filter(Boolean);
}

// ── Public API ───────────────────────────────────────────────────────────────

async function buildLaps(outDir) {
  const pts = buildTrack();

  const refProfile = computeSpeedProfile(pts);
  const curProfile = computeSpeedProfile(pts, {
    lateBraking:  1.0,   // late braking at T1 & T5
    earlyThrottle: 0.8,  // early throttle at chicane
    understeer:   0.9,   // understeer at T6 sweeper
  });

  console.log("Simulating reference lap…");
  const refFrames = simulateLap(pts, refProfile);

  console.log("Simulating current lap…");
  const curFrames = simulateLap(pts, curProfile);

  const refMcap = path.join(outDir, "reference.mcap");
  const curMcap = path.join(outDir, "current.mcap");

  console.log("Writing reference.mcap…");
  await writeMcap(refFrames, refMcap);
  console.log("Writing current.mcap…");
  await writeMcap(curFrames, curMcap);

  const refLapTime = refFrames[refFrames.length - 1].t;
  const curLapTime = curFrames[curFrames.length - 1].t;

  console.log(`Reference lap: ${refLapTime.toFixed(3)}s  Current lap: ${curLapTime.toFixed(3)}s  Delta: +${(curLapTime - refLapTime).toFixed(3)}s`);

  return {
    reference: { frames: refFrames, lapTime: refLapTime, mcap: refMcap },
    current:   { frames: curFrames, lapTime: curLapTime, mcap: curMcap },
    sectors: {
      reference: computeSectors(refFrames),
      current:   computeSectors(curFrames),
    },
    trackLength: TRACK_LEN,
  };
}

module.exports = { buildLaps, computeSectors };
