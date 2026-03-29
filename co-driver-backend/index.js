'use strict';

const express   = require('express');
const cors      = require('cors');
const path      = require('path');
const fs        = require('fs');
const Anthropic = require('@anthropic-ai/sdk');

const { generate }             = require('./data/generate');
const { loadRealLaps }         = require('./data/real_laps');
const { writeMcap }            = require('./data/mcap');
const { extractVideoFromMcap } = require('./data/video');
const { startFoxgloveServer }  = require('./data/foxglove');

const REF_MCAP    = path.join(__dirname, 'reference.mcap');
const CUR_MCAP    = path.join(__dirname, 'current.mcap');
const GEN_VIDEO   = path.join(__dirname, 'generated.mp4');

// ── Mock coaching tips (used when Anthropic key is absent or call fails) ──────

const MOCK_TIPS = [
  { sectorId: 1, tip: 'Good sector, maintain current braking points.', priority: 'low' },
  { sectorId: 2, tip: 'You are braking 8 metres too late into Turn 4. Move your braking marker earlier to rotate the car better and carry more mid-corner speed.', priority: 'high' },
  { sectorId: 3, tip: 'Early throttle application is causing understeer through the fast left. Wait for the apex before applying throttle to allow the front to load up.', priority: 'high' },
  { sectorId: 4, tip: 'Minor oversteer on exit — reduce throttle gradient slightly. Target 0.8s earlier throttle application than current.', priority: 'medium' },
  { sectorId: 5, tip: 'Clean sector. Tyre management is good here, keep consistent.', priority: 'low' },
];

// ── Anthropic coaching ────────────────────────────────────────────────────────

async function fetchCoachingTips(sectorData) {
  const key = process.env.ANTHROPIC_API_KEY;
  if (!key) return { tips: MOCK_TIPS, source: 'MOCK' };

  try {
    const client = new Anthropic.default({ apiKey: key });

    const userPrompt = `Here is the sector comparison data for a racing lap analysis:\n\n${JSON.stringify(sectorData, null, 2)}\n\nReturn a JSON array of 5 coaching tips, one per sector, using this exact shape:\n[{ "sectorId": 1, "tip": "...", "priority": "high|medium|low" }]`;

    const response = await client.messages.create({
      model:      'claude-sonnet-4-0',
      max_tokens: 1000,
      system:     'You are an expert motorsport driving coach. Analyse the sector data and provide specific, actionable coaching tips for a racing driver. Be direct and technical. Use motorsport terminology. Respond ONLY with a valid JSON array.',
      messages:   [{ role: 'user', content: userPrompt }],
    });

    const text = response.content.find(b => b.type === 'text')?.text ?? '';
    const match = text.match(/\[[\s\S]*\]/);
    if (!match) throw new Error('No JSON array in response');
    return { tips: JSON.parse(match[0]), source: 'LIVE' };
  } catch (err) {
    console.warn('Anthropic call failed, using mock tips:', err.message);
    return { tips: MOCK_TIPS, source: 'MOCK (fallback)' };
  }
}

// ── Startup ────────────────────────────────────────────────────────────────────

async function start() {
  console.log('Co-Driver backend starting…');

  // 1. Load lap data (real hackathon data preferred, simulated fallback)
  let data;
  let dataSource;
  try {
    data = loadRealLaps();
    dataSource = 'REAL (hackathon NPY)';
    console.log('  [REAL DATA] Hackathon NPY files loaded successfully.');
  } catch (err) {
    console.warn(`  [SIMULATED] Real data unavailable (${err.message}), generating synthetic laps…`);
    data = generate();
    dataSource = 'SIMULATED';
  }
  const { reference, current, sectorBoundaries, refLapTime, curLapTime, refTimes, curTimes } = data;

  // 2. Write MCAP files
  try {
    console.log('  Writing reference.mcap…');
    await writeMcap(reference, REF_MCAP, true);
    console.log('  Writing current.mcap…');
    await writeMcap(current, CUR_MCAP, false);
    console.log('  MCAP files written: reference.mcap, current.mcap');
  } catch (err) {
    console.warn('  MCAP write failed (server will still run):', err.message);
  }

  // 2b. Extract video from MCAP camera frames
  console.log('  Video extraction: IN PROGRESS…');
  extractVideoFromMcap(CUR_MCAP, GEN_VIDEO)
    .then(() => console.log('  Video extraction: READY → generated.mp4'))
    .catch(err => console.warn(`  Video extraction: FAILED — ${err.message}`));

  // 3. Build sector data (used for coaching prompt)
  const sectorData = sectorBoundaries.map(b => {
    const { id, startIndex, endIndex } = b;
    const refTime = +(Number(refTimes[endIndex]) - Number(refTimes[startIndex])).toFixed(3);
    const curTime = +(Number(curTimes[endIndex]) - Number(curTimes[startIndex])).toFixed(3);
    return { sectorId: id, refTime, curTime, delta: +(curTime - refTime).toFixed(3) };
  });

  // 4. Fetch coaching tips (once, cached)
  console.log('  Fetching coaching tips…');
  const { tips: coachingTips, source: coachingSource } = await fetchCoachingTips(sectorData);

  // 5. Start Express
  const app = express();
  app.use(cors({ origin: ['http://localhost:5173', 'http://127.0.0.1:5173', 'http://localhost:8080', 'http://127.0.0.1:8080'] }));
  app.use(express.json());

  // Attach shared state to app.locals
  app.locals.reference        = reference;
  app.locals.current          = current;
  app.locals.sectorBoundaries = sectorBoundaries;
  app.locals.refLapTime       = refLapTime;
  app.locals.curLapTime       = curLapTime;
  app.locals.refTimes         = refTimes;
  app.locals.curTimes         = curTimes;
  app.locals.coachingTips     = coachingTips;

  // Routes
  app.use('/api/laps',     require('./routes/laps'));
  app.use('/api/sectors',  require('./routes/sectors'));
  app.use('/api/coaching', require('./routes/coaching'));
  app.use('/api/summary',  require('./routes/summary'));

  // Video — serves generated.mp4; Express handles HTTP range requests for seeking
  app.get('/api/video', (req, res) => {
    if (!fs.existsSync(GEN_VIDEO)) {
      return res.status(404).json({ error: 'Video not yet generated. Check backend logs.' });
    }
    res.sendFile(GEN_VIDEO);
  });

  // MCAP download
  app.get('/api/download/:lap', (req, res) => {
    const lap = req.params.lap;
    if (lap !== 'reference' && lap !== 'current') {
      return res.status(400).json({ error: "lap must be 'reference' or 'current'" });
    }
    const file = lap === 'reference' ? REF_MCAP : CUR_MCAP;
    if (!fs.existsSync(file)) return res.status(404).json({ error: 'MCAP file not found' });
    res.setHeader('Content-Type', 'application/octet-stream');
    res.setHeader('Content-Disposition', `attachment; filename="${lap}.mcap"`);
    res.sendFile(file);
  });

  // Health
  app.get('/api/health', (_req, res) => res.json({
    status: 'ready',
    refLapTime,
    curLapTime,
    delta: +(curLapTime - refLapTime).toFixed(3),
    uptime: process.uptime(),
  }));

  // Global error handler
  app.use((err, _req, res, _next) => {
    console.error(err);
    res.status(500).json({ error: err.message });
  });

  // 6. Start Foxglove WebSocket server
  startFoxgloveServer(reference, current);

  app.listen(3000, () => {
    console.log('\nCo-Driver backend running on http://localhost:3000');
    console.log(`Foxglove WebSocket:  ws://localhost:8765`);
    console.log(`Data source: ${dataSource}`);
    console.log(`MCAP files written: reference.mcap, current.mcap`);
    console.log(`Anthropic coaching: ${coachingSource}`);
    console.log(`Video file: ${fs.existsSync(GEN_VIDEO) ? 'READY' : 'generating in background…'}`);
    console.log(`Lap times: REF ${refLapTime}s  CUR ${curLapTime}s  Δ +${(curLapTime - refLapTime).toFixed(3)}s\n`);
  });
}

start().catch(err => {
  console.error('Fatal startup error:', err);
  process.exit(1);
});
