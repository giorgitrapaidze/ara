'use strict';

const { FoxgloveServer } = require('@foxglove/ws-protocol');
const { WebSocketServer } = require('ws');

const FOXGLOVE_PORT = 8765;

// ── Schemas ───────────────────────────────────────────────────────────────────

const TELEMETRY_SCHEMA = JSON.stringify({
  type: 'object',
  properties: {
    lap_type:  { type: 'string', description: '"reference" or "current"' },
    timestamp: { type: 'number' },
    x: { type: 'number' }, y: { type: 'number' },
    speed:    { type: 'number' },
    throttle: { type: 'number' },
    brake:    { type: 'number' },
    steering: { type: 'number' },
    rpm:      { type: 'integer' },
    gear:     { type: 'integer' },
    sector:   { type: 'integer' },
    yaw:      { type: 'number' },
    tempFL: { type: 'number' }, tempFR: { type: 'number' },
    tempRL: { type: 'number' }, tempRR: { type: 'number' },
    loadFL: { type: 'number' }, loadFR: { type: 'number' },
    loadRL: { type: 'number' }, loadRR: { type: 'number' },
  },
});

const CAMERA_SCHEMA = JSON.stringify({
  title: 'foxglove.CompressedImage',
  type: 'object',
  properties: {
    timestamp: {
      type: 'object',
      properties: { sec: { type: 'integer' }, nsec: { type: 'integer' } },
    },
    frame_id: { type: 'string' },
    data:     { type: 'string', contentEncoding: 'base64' },
    format:   { type: 'string' },
  },
});

// ── Camera frame pre-generation ───────────────────────────────────────────────

function generateCameraFrames(rows) {
  let createCanvas;
  try { ({ createCanvas } = require('canvas')); } catch (_) { return []; }

  const W = 640, H = 360;
  const frames = [];

  for (let i = 0; i < rows.length; i += 6) {  // 60 Hz → 10 Hz
    const r = rows[i];
    const canvas = createCanvas(W, H);
    const ctx    = canvas.getContext('2d');

    // Background
    ctx.fillStyle = '#0d0d0d';
    ctx.fillRect(0, 0, W, H);

    // Track surface hint
    ctx.fillStyle = '#1a2a1a';
    ctx.fillRect(0, 0, W, H - 90);

    // HUD bar
    ctx.fillStyle = 'rgba(0,0,0,0.8)';
    ctx.fillRect(0, H - 90, W, 90);

    // Speed
    ctx.fillStyle = '#ffffff';
    ctx.font = 'bold 36px monospace';
    ctx.fillText(`${Math.round(r.speed)} km/h`, 16, H - 50);

    // Gear
    ctx.fillStyle = '#ffcc00';
    ctx.font = 'bold 36px monospace';
    ctx.fillText(`G${r.gear}`, 230, H - 50);

    // Throttle bar (green)
    ctx.fillStyle = '#222';
    ctx.fillRect(290, H - 75, 200, 18);
    ctx.fillStyle = '#00e676';
    ctx.fillRect(290, H - 75, Math.round(r.throttle * 200), 18);

    // Brake bar (red)
    ctx.fillStyle = '#222';
    ctx.fillRect(290, H - 50, 200, 18);
    ctx.fillStyle = '#ff1744';
    ctx.fillRect(290, H - 50, Math.round(r.brake * 200), 18);

    // Labels
    ctx.fillStyle = '#888';
    ctx.font = '12px monospace';
    ctx.fillText('THR', 295, H - 80);
    ctx.fillText('BRK', 295, H - 55);

    // Sector
    ctx.fillStyle = '#00bcd4';
    ctx.font = 'bold 20px monospace';
    ctx.fillText(`S${r.sector}`, W - 55, H - 60);

    frames.push({
      tsNs: r.timestamp,
      jpeg: canvas.toBuffer('image/jpeg', { quality: 0.7 }),
    });
  }

  return frames;
}

// ── Main export ───────────────────────────────────────────────────────────────

function startFoxgloveServer(reference, current) {
  // Pre-generate camera frames for the current lap at startup
  console.log('[Foxglove] Generating camera frames…');
  const cameraFrames = generateCameraFrames(current);
  console.log(cameraFrames.length > 0
    ? `[Foxglove] ${cameraFrames.length} camera frames ready`
    : '[Foxglove] canvas unavailable — /racing/camera will be silent');

  const foxglove = new FoxgloveServer({ name: 'Co-Driver' });
  const wss = new WebSocketServer({ port: FOXGLOVE_PORT, host: '0.0.0.0' });

  wss.on('connection', (ws, req) => {
    foxglove.handleConnection(ws, req.url ?? '/');
    console.log('[Foxglove] client connected');
  });

  // Advertise channels
  const telChan = foxglove.addChannel({
    topic: '/racing/telemetry',
    encoding: 'json',
    schemaName: 'RacingTelemetry',
    schema: TELEMETRY_SCHEMA,
  });

  const camChan = foxglove.addChannel({
    topic: '/racing/camera',
    encoding: 'json',
    schemaName: 'foxglove.CompressedImage',
    schema: CAMERA_SCHEMA,
  });

  // Merged timeline: both laps interleaved and sorted by timestamp
  const timeline = [
    ...reference.map(r => ({ row: r, lap_type: 'reference' })),
    ...current.map(r => ({ row: r, lap_type: 'current' })),
  ].sort((a, b) => (a.row.timestamp < b.row.timestamp ? -1 : 1));

  // Playback state
  const subscriptions = new Set();
  let intervalId = null;
  let tlIdx      = 0;
  let camIdx     = 0;
  let lapStartMs = 0;

  foxglove.on('subscribe', (chanId) => {
    subscriptions.add(chanId);
    if (subscriptions.size === 1) {
      tlIdx = 0; camIdx = 0; lapStartMs = Date.now();
      intervalId = setInterval(tick, 16); // ~60 Hz
      console.log('[Foxglove] Playback started');
    }
  });

  foxglove.on('unsubscribe', (chanId) => {
    subscriptions.delete(chanId);
    if (subscriptions.size === 0 && intervalId) {
      clearInterval(intervalId);
      intervalId = null;
      console.log('[Foxglove] Playback stopped');
    }
  });

  foxglove.on('error', (err) => console.error('[Foxglove] error:', err.message));

  function tick() {
    const elapsedNs = BigInt(Math.round((Date.now() - lapStartMs) * 1e6));

    // Drain telemetry events up to now
    while (tlIdx < timeline.length && timeline[tlIdx].row.timestamp <= elapsedNs) {
      const { row, lap_type } = timeline[tlIdx++];
      foxglove.sendMessage(telChan, row.timestamp, Buffer.from(JSON.stringify({
        lap_type,
        timestamp: Number(row.timestamp) / 1e9,
        x: row.x, y: row.y,
        speed: row.speed, throttle: row.throttle,
        brake: row.brake, steering: row.steering,
        rpm: row.rpm, gear: row.gear,
        sector: row.sector, yaw: row.yaw,
        tempFL: row.tempFL, tempFR: row.tempFR,
        tempRL: row.tempRL, tempRR: row.tempRR,
        loadFL: row.loadFL, loadFR: row.loadFR,
        loadRL: row.loadRL, loadRR: row.loadRR,
      })));
    }

    // Drain camera frames (current lap, 10 Hz)
    if (cameraFrames.length > 0) {
      while (camIdx < cameraFrames.length && cameraFrames[camIdx].tsNs <= elapsedNs) {
        const { tsNs, jpeg } = cameraFrames[camIdx++];
        foxglove.sendMessage(camChan, tsNs, Buffer.from(JSON.stringify({
          timestamp: { sec: Number(tsNs / 1_000_000_000n), nsec: Number(tsNs % 1_000_000_000n) },
          frame_id: 'camera',
          data: jpeg.toString('base64'),
          format: 'jpeg',
        })));
      }
    }

    // Loop when all streams exhausted
    if (tlIdx >= timeline.length) {
      tlIdx = 0; camIdx = 0; lapStartMs = Date.now();
    }
  }

  console.log(`[Foxglove] WebSocket server on ws://localhost:${FOXGLOVE_PORT}`);
}

module.exports = { startFoxgloveServer };
