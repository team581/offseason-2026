// Run with Node and wpilog-parser 3.2.1 (see docs/swerve-lag-q64.md).
import { readFileSync } from 'node:fs';
import { pathToFileURL } from 'node:url';

const [logPath, parserModule] = process.argv.slice(2);
if (!logPath || !parserModule) {
  throw new Error('Usage: node tools/analyze_swerve_lag.mjs LOG PATH_TO_WPILOG_PARSER_DIST_INDEX');
}
const { parseDataLog, isDataRecord } = await import(pathToFileURL(parserModule).href);
const names = ['DS:enabled', 'DS:autonomous', 'Robot/Swerve/ModuleTargets',
  'Robot/Swerve/FieldRelativeSpeeds', 'Robot/Localization/EstimatedPose'];
const series = Object.fromEntries(names.map(name => [name, []]));
for (const record of parseDataLog(readFileSync(logPath))) {
  if (!isDataRecord(record)) continue;
  const key = record.name.slice(1);
  if (key in series) series[key].push([Number(record.timestamp) / 1e6, record.payload]);
}
for (const samples of Object.values(series)) samples.sort((a, b) => a[0] - b[0]);
function indexAt(samples, time) {
  let lo = 0, hi = samples.length;
  while (lo < hi) {
    const mid = (lo + hi) >> 1;
    if (samples[mid][0] <= time) lo = mid + 1;
    else hi = mid;
  }
  return Math.max(0, lo - 1);
}
const valueAt = (samples, time) => samples[indexAt(samples, time)][1];
// The symmetric comp chassis makes center translation the mean wheel vector.
// No angular tau is estimated here; that needs the original module locations.
const targets = series['Robot/Swerve/ModuleTargets'].map(([time, modules]) => {
  const heading = valueAt(series['Robot/Localization/EstimatedPose'], time).rotation.value;
  const x = modules.reduce((s, m) => s + m.speed * Math.cos(m.angle.value), 0) / 4;
  const y = modules.reduce((s, m) => s + m.speed * Math.sin(m.angle.value), 0) / 4;
  return [time, [x * Math.cos(heading) - y * Math.sin(heading),
    x * Math.sin(heading) + y * Math.cos(heading)]];
});
function command(time, linear = true) {
  const i = indexAt(targets, time);
  const [start, u] = targets[i];
  const next = targets[Math.min(i + 1, targets.length - 1)];
  const fraction = linear && next[0] > start ? (time - start) / (next[0] - start) : 0;
  return u.map((x, j) => x + (next[1][j] - x) * fraction);
}
const measured = series['Robot/Swerve/FieldRelativeSpeeds'].map(([t, v]) => ({
  t, v: [v.vx, v.vy], auto: valueAt(series['DS:autonomous'], t),
}));
const rows = [];
const rejected = { disabled: 0, sampleGap: 0, smallError: 0, largeError: 0 };
for (let i = 0; i < measured.length - 1; i++) {
  const a = measured[i], b = measured[i + 1], dt = b.t - a.t;
  if (!valueAt(series['DS:enabled'], a.t) || !valueAt(series['DS:enabled'], b.t)) {
    rejected.disabled++; continue;
  }
  if (dt < 0.01 || dt > 0.25) { rejected.sampleGap++; continue; }
  const u = command(a.t);
  const error = Math.hypot(...u.map((x, j) => x - a.v[j]));
  if (error < 0.25) { rejected.smallError++; continue; }
  if (error > 3) { rejected.largeError++; continue; }
  const kind = a.v[0] * u[0] + a.v[1] * u[1] < -0.1 ? 'reversal'
    : Math.hypot(...u) < Math.hypot(...a.v) - 0.15 ? 'braking'
    : Math.hypot(...a.v) < 0.25 ? 'start' : 'speed change';
  // Hold out whole five-second blocks, not random neighboring samples.
  rows.push({ a, b, dt, kind, fold: Math.floor(a.t / 5) % 3 });
}
function score(samples, tau, delay = 0, linear = true) {
  if (!samples.length) return { n: 0, rmse: null, measuredOnlyRmse: null };
  let squared = 0, baseline = 0;
  for (const row of samples) {
    let prediction = row.a.v.slice();
    const steps = 20, dt = row.dt / steps, decay = Math.exp(-dt / tau);
    for (let k = 0; k < steps; k++) {
      const u = command(row.a.t + (k + 0.5) * dt + delay, linear);
      prediction = prediction.map((x, j) => u[j] + (x - u[j]) * decay);
    }
    for (let j = 0; j < 2; j++) {
      squared += (row.b.v[j] - prediction[j]) ** 2;
      baseline += (row.b.v[j] - row.a.v[j]) ** 2;
    }
  }
  return { n: samples.length, rmse: Math.sqrt(squared / samples.length),
    measuredOnlyRmse: Math.sqrt(baseline / samples.length) };
}
function fit(samples, delay = 0, linear = true) {
  let best = { rmse: Infinity };
  for (let tick = 2; tick <= 80; tick++) {
    const tau = tick / 100;
    const result = score(samples, tau, delay, linear);
    if (result.rmse !== null && result.rmse < best.rmse) best = { tau, ...result };
  }
  return best;
}
const train = rows.filter(r => r.fold !== 2), test = rows.filter(r => r.fold === 2);
const quantile = (a, p) => a.toSorted((x, y) => x - y)[Math.floor((a.length - 1) * p)];
const intervals = measured.slice(1).map((m, i) => m.t - measured[i].t);
const comparison = samples => ({ old: score(samples, 0.15), proposed: score(samples, 0.30) });
const stable = rows.filter(row => {
  const before = command(row.a.t - 0.2), current = command(row.a.t), after = command(row.b.t);
  return Math.hypot(...before.map((x, j) => x - current[j])) < 0.35
    && Math.hypot(...after.map((x, j) => x - current[j])) < 0.35;
});
const report = {
  logPath, measuredSamples: measured.length, targetSamples: targets.length,
  intervalQuantilesSeconds: [0, 0.1, 0.5, 0.9, 1].map(p => quantile(intervals, p)),
  rejected, trainFit: fit(train), train: comparison(train), heldOut: comparison(test),
  approximatelySteadyCommands: { fit: fit(stable), ...comparison(stable) },
  maneuvers: Object.fromEntries(['start', 'speed change', 'braking', 'reversal'].map(kind => [kind, {
    fit: fit(rows.filter(r => r.kind === kind)),
    heldOut: comparison(test.filter(r => r.kind === kind)),
  }])),
  modes: Object.fromEntries([true, false].map(auto => [auto ? 'auto' : 'teleop',
    fit(rows.filter(r => r.a.auto === auto))])),
  timingSensitivity: [-0.1, -0.05, 0, 0.05, 0.1].map(delay => ({ delay,
    linear: fit(train, delay, true), held: fit(train, delay, false) })),
};
console.log(JSON.stringify(report, null, 2));
