/** Offline review only. Never opens a serial/BLE connection or modifies the source exports. */
import assert from 'node:assert/strict';
import {readFile, mkdir, copyFile, writeFile} from 'node:fs/promises';
import {constants} from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve, basename, join} from 'node:path';
import {fileURLToPath} from 'node:url';
import {parseAdaptiveLog, ADAPTIVE_COLUMNS} from '../web-bluetooth-console/lib/exo-adaptive.ts';
import {parseLockRecords} from '../web-bluetooth-console/lib/exo-adaptive-capture.ts';

const [logArg, markersArg] = process.argv.slice(2);
if (!logArg || !markersArg) throw new Error('Usage: node tools/review_manual_afo_capture.mjs <log> <markers.json>');
const root = fileURLToPath(new URL('../', import.meta.url));
const logPath = resolve(logArg), markersPath = resolve(markersArg);
const [raw, metadataBytes] = await Promise.all([readFile(logPath), readFile(markersPath)]);
const manifest = JSON.parse(metadataBytes.toString('utf8'));
const sha = bytes => createHash('sha256').update(bytes).digest('hex');
const date = new Intl.DateTimeFormat('en-CA', {timeZone:'Asia/Shanghai',year:'numeric',month:'2-digit',day:'2-digit'}).format(new Date()).replaceAll('-','');
const out = join(root, 'output/adaptive', date, `manual_review_${Date.now()}`);
await mkdir(out, {recursive:true});
for (const path of [logPath, markersPath]) await copyFile(path, join(out, basename(path)), constants.COPYFILE_EXCL);
assert.equal(raw.length, manifest.bytes, 'raw byte count differs from manifest');
const text = raw.toString('utf8');
const parsed = parseAdaptiveLog(text), locks = parseLockRecords(text);
assert.equal(parsed.malformed, 0, 'malformed structured records');
const boundary = manifest.summary.boundary_us;
assert.ok(Number.isSafeInteger(boundary) && boundary > 0);
const rows = new Map(); let historical = 0, duplicates = 0;
for (const row of parsed.rows) {
  if (row.timestamp_us < boundary) {historical++; continue;}
  const key = `${row.timestamp_us}:${row.side}`;
  const old = rows.get(key);
  if (old) {
    assert.ok(ADAPTIVE_COLUMNS.every(k => k === 'dropped' || old[k] === row[k]), `conflict at ${key}`);
    duplicates++;
  }
  rows.set(key, row);
}
const data = [...rows.values()].sort((a,b) => a.timestamp_us-b.timestamp_us || a.side-b.side);
assert.equal(data.length, manifest.summary.unique_rows);
const preflightExcluded = text.split(/ADAPT max_execution_us=[^\n]*\n/).reduce((sum, block) => {
  const snapshot = parseLockRecords(block).find(r=>r.kind==='ADLOCKSTATE');
  return sum + (snapshot && snapshot.timestamp_us < boundary ? parseAdaptiveLog(block).rows.length : 0);
},0);
assert.equal(historical, manifest.summary.historical_rows_excluded + preflightExcluded);
const times = [...new Set(data.map(r=>r.timestamp_us))];
assert.ok(times.every(t=>rows.has(`${t}:0`) && rows.has(`${t}:1`)), 'unpaired ADAPT records');
const events = locks.filter(r=>r.kind==='ADLOCK' && r.timestamp_us>=boundary);
assert.equal(events.length, manifest.summary.exact_loss_events);
const states = locks.filter(r=>r.kind==='ADLOCKSTATE' && r.timestamp_us>=boundary);
for (const side of [0,1]) {
  const series = states.filter(r=>r.side===side);
  assert.ok(series.every((r,i)=>i===0 || r.timestamp_us>series[i-1].timestamp_us), 'current snapshot time did not advance');
}
const uniques = values => [...new Set(values)];
const range = values => values.length ? [Math.min(...values), Math.max(...values)] : null;
const blocks = [...text.matchAll(/===== Exoskeleton System =====([\s\S]*?)===== Product BLE =====/g)].map(m=>m[1]);
const field = (s,name) => s.match(new RegExp(`^\\s*${name}:\\s*([^\\r\\n]+)`, 'm'))?.[1].trim() ?? null;
const postInitial = raw.subarray(manifest.initial_connection.bytes_before_readback).toString('utf8');
const resetPattern = /ESP-ROM:|rst:0x|Brownout detector was triggered|Guru Meditation|Rebooting/g;
const sampleStats = subset => ({rows:subset.length, valid_input_rows:subset.filter(r=>r.feedback_valid===1 && !(r.faults&1)).length,
  locked_rows:subset.filter(r=>r.locked===1).length, q_range_deg:range(subset.map(r=>r.q_deg)),
  velocity_range_deg_s:range(subset.map(r=>r.v_deg_s)), faults:uniques(subset.map(r=>r.faults)),
  accepted_events_range:range(subset.map(r=>r.accepted_events)), frequency_range_hz:range(subset.map(r=>r.hz))});
const windows=[]; let start=null;
for (const marker of manifest.markers) {
  if (marker.label==='开始活动') start=marker;
  if (marker.label==='停止活动' && start) {
    windows.push({index:windows.length+1, start_host_iso:start.host_iso,stop_host_iso:marker.host_iso,
      host_duration_s:(marker.elapsed_ms-start.elapsed_ms)/1000,
      approximate_mcu_bounds_us:[start.last_snapshot_us,marker.last_snapshot_us],
      note:'MCU bounds are last-received snapshots attached to human clicks, not synchronized motion times.',
      sides:[0,1].map(side=>({side,...sampleStats(data.filter(r=>r.side===side && r.timestamp_us>=start.last_snapshot_us && r.timestamp_us<marker.last_snapshot_us))}))});
    start=null;
  }
}
const valid = data.filter(r=>r.feedback_valid===1 && !(r.faults&1));
const summary = {
  scope:'Offline review of user-exported raw UART and manual markers; no device access',
  provenance:{log:{original:logPath,sha256:sha(raw),bytes:raw.length},markers:{original:markersPath,sha256:sha(metadataBytes)},
    reviewer_sha256:sha(await readFile(fileURLToPath(import.meta.url)))},
  capture:{start_host_iso:manifest.started_host_iso,end_host_iso:manifest.ended_host_iso,complete_as_recorded:manifest.complete,
    boots:uniques([...text.matchAll(/Boot ID:\s*(0x[0-9A-Fa-f]{8})/g)].map(m=>m[1].toUpperCase())),
    reset_reason_lines:text.split(/\r?\n/).filter(line=>/rst:0x/.test(line)),
    reset_signatures_after_initial: [...postInitial.matchAll(resetPattern)].map(m=>m[0]),
    bytes:raw.length,parsed_rows:parsed.rows.length,malformed_rows:parsed.malformed,unique_rows:data.length,
    historical_rows_excluded:historical,preflight_rows_excluded:preflightExcluded,
    drained_history_rows_excluded:manifest.summary.historical_rows_excluded,
    duplicates_removed:duplicates,paired_timestamps:times.length,
    boundary_us:boundary,last_us:times.at(-1),span_s:(times.at(-1)-boundary)/1e6,
    sample_gap_ms_range:range(times.slice(1).map((t,i)=>(t-times[i])/1000)),
    system_state:uniques(data.map(r=>r.system_state)),command_valid:uniques(data.map(r=>r.command_valid)),
    final_ff_range_nm:range(data.map(r=>r.final_ff_command_nm)),adapt_dropped_range:range(data.map(r=>r.dropped)),
    exact_lock_losses:events.length,current_lock_reasons:uniques(states.map(r=>r.values[6]))},
  system_readbacks:{count:blocks.length,states:uniques(blocks.map(b=>field(b,'State'))),start_blocks:uniques(blocks.map(b=>field(b,'Start block'))),
    ready:uniques(blocks.map(b=>field(b,'Ready to start'))),run_requested:uniques(blocks.map(b=>field(b,'Run requested'))),
    output_allowed:uniques(blocks.map(b=>field(b,'Output allowed'))),start_counters:uniques(blocks.map(b=>field(b,'START'))),
    stop_counters:uniques(blocks.map(b=>field(b,'STOP')))},
  sides:[0,1].map(side=>({side,...sampleStats(data.filter(r=>r.side===side))})),
  activity_windows:windows,
  direction_projection:{performed:false,eligible_valid_locked_moving_rows:data.filter(r=>r.feedback_valid===1 && !(r.faults&1) && r.locked===1 && Math.abs(r.v_deg_s)>=3).length,
    reason:valid.length ? 'This intake report does not project curves; inspect valid locked rows separately.' : 'No valid AFO input. Zero-filled invalid outputs cannot represent physical stillness or be used for phase/torque-direction projection.'},
  limitations:['Completed transport capture is not usable-motion-data acceptance.', 'Zero lock-loss events are not evidence of stable lock when locked is never true.',
    'The LEFT_ZERO_INVALID priority block proves the left zero is invalid; the UART export does not contain per-side raw motor diagnostics.',
    'The initial POWERON report does not by itself identify USB connection versus external power as the cause.',
    'Historical readback is not current hardware state.']
};
await writeFile(join(out,'summary.json'),JSON.stringify(summary,null,2)+'\n',{flag:'wx'});
await writeFile(join(out,'deduplicated_adapt.csv'),ADAPTIVE_COLUMNS.join(',')+'\n'+data.map(r=>ADAPTIVE_COLUMNS.map(k=>r[k]).join(',')).join('\n')+'\n',{flag:'wx'});
console.log(JSON.stringify({out,...summary.capture,systems:summary.system_readbacks,sides:summary.sides,activity_windows:summary.activity_windows,projection:summary.direction_projection},null,2));
