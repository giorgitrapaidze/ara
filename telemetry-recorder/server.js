"use strict";

const express = require("express");
const fs = require("fs");
const path = require("path");
const { McapWriter } = require("@mcap/core");

const app = express();
app.use(express.json());
app.use(express.static(path.join(__dirname, "public")));

const OUTPUT_FILE = path.join(__dirname, "telemetry.mcap");
const HZ = 60;

// ---------------------------------------------------------------------------
// MCAP helpers
// ---------------------------------------------------------------------------

class FileWritable {
  constructor(filePath) {
    this.fd = fs.openSync(filePath, "w");
    this._pos = 0n;
  }
  async write(buf) {
    fs.writeSync(this.fd, buf);
    this._pos += BigInt(buf.byteLength);
  }
  position() {
    return this._pos;
  }
  close() {
    fs.closeSync(this.fd);
  }
}

function jsonSchema(name, properties) {
  return {
    name,
    encoding: "jsonschema",
    data: Buffer.from(
      JSON.stringify({ type: "object", properties }),
      "utf8"
    ),
  };
}

// ---------------------------------------------------------------------------
// Simulation state
// ---------------------------------------------------------------------------

const TRACK = buildTrack();

function buildTrack() {
  // Rough figure-8-ish circuit: array of {curvature, length} segments.
  // Curvature > 0 = left turn, < 0 = right, 0 = straight.
  const segments = [
    { curvature: 0,     length: 300 },  // back straight
    { curvature:  0.02, length: 200 },  // hairpin left
    { curvature: 0,     length: 150 },  // mini straight
    { curvature: -0.015,length: 180 },  // right sweeper
    { curvature: 0,     length: 400 },  // main straight
    { curvature:  0.025,length: 160 },  // tight left
    { curvature: -0.01, length: 220 },  // long right
    { curvature: 0,     length: 120 },  // short straight
    { curvature:  0.018,length: 170 },  // chicane left
    { curvature: -0.018,length: 170 },  // chicane right
    { curvature: 0,     length: 200 },  // pit straight
  ];

  // Pre-compute x,y,heading along the track
  const points = [];
  let x = 0, y = 0, heading = 0;
  const STEP = 2; // metres per sample

  for (const seg of segments) {
    const steps = Math.round(seg.length / STEP);
    for (let i = 0; i < steps; i++) {
      points.push({ x, y, heading, curvature: seg.curvature });
      heading += seg.curvature * STEP;
      x += Math.cos(heading) * STEP;
      y += Math.sin(heading) * STEP;
    }
  }

  // Speed target per point: lower in corners, higher on straights
  for (const p of points) {
    const absCurv = Math.abs(p.curvature);
    if (absCurv === 0)       p.targetSpeed = 280 + Math.random() * 20;
    else if (absCurv < 0.012) p.targetSpeed = 160 + Math.random() * 20;
    else if (absCurv < 0.02)  p.targetSpeed = 110 + Math.random() * 15;
    else                       p.targetSpeed = 75  + Math.random() * 15;
  }

  return points;
}

const sim = {
  idx: 0,
  speed: 100,   // km/h
  rpm: 6000,
  gear: 3,
  throttle: 0,
  brake: 0,
  steer: 0,
  latG: 0,
  lonG: 0,
  vertG: 1.0,
};

const GEAR_UP_RPM   = [0, 9500, 9200, 9000, 8800, 8600, 8400, 9999];
const GEAR_DOWN_RPM = [0,    0, 5500, 5200, 5000, 4800, 4600, 4400];
const GEAR_RATIOS   = [0, 3.5, 2.5, 1.9, 1.5, 1.25, 1.1, 1.0];
const FINAL_DRIVE   = 3.8;
const WHEEL_CIRC    = 1.9; // metres

function speedToRpm(speedKmh, gear) {
  const speedMs = speedKmh / 3.6;
  const wheelRps = speedMs / WHEEL_CIRC;
  return wheelRps * GEAR_RATIOS[gear] * FINAL_DRIVE * 60;
}

function tickSimulation() {
  const dt = 1 / HZ;
  const pt  = TRACK[sim.idx];
  const next = TRACK[(sim.idx + 1) % TRACK.length];

  // Look-ahead braking: find min target speed in next N points
  const lookahead = 40;
  let minAhead = pt.targetSpeed;
  for (let i = 1; i <= lookahead; i++) {
    const ahead = TRACK[(sim.idx + i) % TRACK.length].targetSpeed;
    if (ahead < minAhead) minAhead = ahead;
  }

  const speedError = minAhead - sim.speed;
  if (speedError > 5) {
    sim.throttle = Math.min(1, sim.throttle + 0.15 * dt * 60);
    sim.brake = 0;
    const accel = sim.throttle * (1 - sim.speed / 320) * 25; // m/s²
    sim.lonG = accel / 9.81;
    sim.speed = Math.min(300, sim.speed + accel * dt * 3.6);
  } else if (speedError < -5) {
    sim.brake = Math.min(1, Math.abs(speedError) / 80);
    sim.throttle = 0;
    const decel = sim.brake * 40;
    sim.lonG = -decel / 9.81;
    sim.speed = Math.max(40, sim.speed - decel * dt * 3.6);
  } else {
    sim.throttle = Math.max(0.1, sim.throttle - 0.05 * dt * 60);
    sim.brake = 0;
    sim.lonG *= 0.85;
  }

  // Gear logic
  sim.rpm = speedToRpm(sim.speed, sim.gear);
  if (sim.gear < 7 && sim.rpm > GEAR_UP_RPM[sim.gear]) sim.gear++;
  if (sim.gear > 1 && sim.rpm < GEAR_DOWN_RPM[sim.gear]) sim.gear--;
  sim.rpm = speedToRpm(sim.speed, sim.gear);
  sim.rpm = Math.max(4000, Math.min(12000, sim.rpm + (Math.random() - 0.5) * 80));

  // Lateral G from curvature × speed²
  const speedMs = sim.speed / 3.6;
  sim.latG = pt.curvature * speedMs * speedMs / 9.81 + (Math.random() - 0.5) * 0.05;
  sim.steer = Math.max(-1, Math.min(1, pt.curvature * 60 + (Math.random() - 0.5) * 0.02));
  sim.vertG = 1.0 + (Math.random() - 0.5) * 0.08;

  // Advance track position proportional to speed
  const distPerTick = speedMs * dt;
  const trackStep = Math.round(distPerTick / 2); // 2 m/sample
  sim.idx = (sim.idx + Math.max(1, trackStep)) % TRACK.length;

  return {
    telemetry: {
      speed_kmh:  +sim.speed.toFixed(2),
      rpm:        Math.round(sim.rpm),
      gear:       sim.gear,
      throttle:   +sim.throttle.toFixed(3),
      brake:      +sim.brake.toFixed(3),
    },
    imu: {
      lat_g:  +sim.latG.toFixed(4),
      lon_g:  +sim.lonG.toFixed(4),
      vert_g: +sim.vertG.toFixed(4),
    },
    pose: {
      x:       +pt.x.toFixed(3),
      y:       +pt.y.toFixed(3),
      heading: +pt.heading.toFixed(5),
    },
    inputs: {
      steering_angle: +sim.steer.toFixed(4),
      throttle:       +sim.throttle.toFixed(3),
      brake:          +sim.brake.toFixed(3),
    },
  };
}

// ---------------------------------------------------------------------------
// Recording session
// ---------------------------------------------------------------------------

let session = null; // { writer, fileWritable, channels, interval, seq }

async function startRecording() {
  if (session) return false;

  const fw = new FileWritable(OUTPUT_FILE);
  const writer = new McapWriter({ writable: fw });
  await writer.start({ library: "telemetry-recorder", profile: "" });

  // Register schemas
  const schemas = {
    telemetry: await writer.registerSchema(jsonSchema("car/Telemetry", {
      speed_kmh: { type: "number" },
      rpm:       { type: "number" },
      gear:      { type: "number" },
      throttle:  { type: "number" },
      brake:     { type: "number" },
    })),
    imu: await writer.registerSchema(jsonSchema("car/Imu", {
      lat_g:  { type: "number" },
      lon_g:  { type: "number" },
      vert_g: { type: "number" },
    })),
    pose: await writer.registerSchema(jsonSchema("car/Pose", {
      x:       { type: "number" },
      y:       { type: "number" },
      heading: { type: "number" },
    })),
    inputs: await writer.registerSchema(jsonSchema("car/Inputs", {
      steering_angle: { type: "number" },
      throttle:       { type: "number" },
      brake:          { type: "number" },
    })),
  };

  // Register channels
  const channels = {
    telemetry: await writer.registerChannel({ topic: "/car/telemetry", messageEncoding: "json", schemaId: schemas.telemetry, metadata: new Map() }),
    imu:       await writer.registerChannel({ topic: "/car/imu",       messageEncoding: "json", schemaId: schemas.imu,       metadata: new Map() }),
    pose:      await writer.registerChannel({ topic: "/car/pose",      messageEncoding: "json", schemaId: schemas.pose,      metadata: new Map() }),
    inputs:    await writer.registerChannel({ topic: "/car/inputs",    messageEncoding: "json", schemaId: schemas.inputs,    metadata: new Map() }),
  };

  let seq = 0;
  const startNs = BigInt(Date.now()) * 1_000_000n;

  const interval = setInterval(async () => {
    const data = tickSimulation();
    const nowNs = BigInt(Date.now()) * 1_000_000n;

    for (const [key, channelId] of Object.entries(channels)) {
      await writer.addMessage({
        channelId,
        sequence: seq++,
        logTime: nowNs,
        publishTime: nowNs,
        data: Buffer.from(JSON.stringify(data[key]), "utf8"),
      });
    }
  }, 1000 / HZ);

  session = { writer, fw, interval };
  console.log("Recording started →", OUTPUT_FILE);
  return true;
}

async function stopRecording() {
  if (!session) return false;
  clearInterval(session.interval);
  await session.writer.end();
  session.fw.close();
  session = null;
  console.log("Recording stopped.");
  return true;
}

// ---------------------------------------------------------------------------
// Routes
// ---------------------------------------------------------------------------

app.post("/start", async (req, res) => {
  if (session) return res.status(409).json({ error: "Already recording" });
  await startRecording();
  res.json({ status: "recording" });
});

app.post("/stop", async (req, res) => {
  if (!session) return res.status(409).json({ error: "Not recording" });
  await stopRecording();
  res.json({ status: "stopped", file: "telemetry.mcap" });
});

app.get("/status", (req, res) => {
  res.json({ recording: !!session });
});

app.get("/live", (req, res) => {
  if (!session) return res.status(404).json({ error: "Not recording" });
  const pt = TRACK[sim.idx];
  res.json({
    telemetry: { speed_kmh: +sim.speed.toFixed(2), rpm: Math.round(sim.rpm), gear: sim.gear, throttle: +sim.throttle.toFixed(3), brake: +sim.brake.toFixed(3) },
    imu:       { lat_g: +sim.latG.toFixed(4), lon_g: +sim.lonG.toFixed(4), vert_g: +sim.vertG.toFixed(4) },
    pose:      { x: +pt.x.toFixed(3), y: +pt.y.toFixed(3), heading: +pt.heading.toFixed(5) },
    inputs:    { steering_angle: +sim.steer.toFixed(4), throttle: +sim.throttle.toFixed(3), brake: +sim.brake.toFixed(3) },
  });
});

app.get("/download", (req, res) => {
  if (session) return res.status(409).json({ error: "Stop recording first" });
  if (!fs.existsSync(OUTPUT_FILE)) return res.status(404).json({ error: "No file yet" });
  res.download(OUTPUT_FILE, "telemetry.mcap");
});

app.listen(3000, () => console.log("Telemetry recorder running at http://localhost:3000"));
