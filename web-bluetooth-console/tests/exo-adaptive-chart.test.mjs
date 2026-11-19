import assert from 'node:assert/strict';
import test from 'node:test';
import { ADAPTIVE_COLUMNS } from '../lib/exo-adaptive.ts';
import { chartLog, orderedChartRows, comparisonTorque, directionPoints, chartSegments } from '../lib/exo-adaptive-chart.ts';

function row(t, side=0, patch={}) {
  return {...Object.fromEntries(ADAPTIVE_COLUMNS.map(k=>[k,0])), timestamp_us:t, side,
    feedback_valid:1, locked:1, generation:1, corrected_phase:.25, ...patch};
}
const line = r => 'ADAPT,' + ADAPTIVE_COLUMNS.map(k=>r[k]).join(',') + '\n';

test('single-Boot cache order is sorted/deduplicated, but mixed or unknown epochs are rejected',()=>{
  const text='Boot ID: 0xF872148D\n'+line(row(200))+line(row(100))+line(row(100,0,{dropped:4}));
  assert.deepEqual(chartLog(text,'capture').rows.map(r=>r.timestamp_us),[100,200]);
  assert.throws(()=>chartLog(text+'Boot ID: 0x11111111\n','mixed'),/多个Boot/);
  assert.throws(()=>chartLog(text.replace(/Boot ID:[^\n]+\n/,''),'unknown'),/时间有回退/);
  assert.throws(()=>chartLog(text+line(row(100,0,{q_deg:1})),'conflict'),/冲突/);
  assert.throws(()=>chartLog(text+'ADAPT,1,2,3\n','broken'),/损坏/);
});

test('comparison uses production C reference values and never edits command values',()=>{
  assert.ok(Math.abs(comparisonTorque(.179203,.1)-(-.07734568))<1e-7);
  assert.ok(Math.abs(comparisonTorque(.250753,5)-(-.09936642*50))<5e-6);
  assert.equal(comparisonTorque(.25,0),0);
  assert.ok(Number.isNaN(comparisonTorque(.25,16)));
  const original=row(100,0,{v_deg_s:20});
  const copy=structuredClone(original);
  const [point]=directionPoints([original],0,5);
  assert.equal(point.opposed,true); assert.equal(point.positionSupport,'unknown');
  assert.equal(point.row.command_valid,0); assert.equal(point.row.final_ff_command_nm,0);
  assert.deepEqual(original,copy);
});

test('phase wrap, invalid input, generation, missing samples and exact losses break plotted lines',()=>{
  for(const changed of [{corrected_phase:.01},{feedback_valid:0},{generation:2}]) {
    const points=directionPoints([row(100000,0,{corrected_phase:.99}),row(200000,0,changed)],0,5);
    assert.ok(chartSegments(points,p=>p.row.corrected_phase,true).every(s=>s.length===1));
  }
  const points=directionPoints([row(100000),row(300000)],0,5);
  assert.equal(chartSegments(points,p=>p.row.q_deg).length,2);
  const close=directionPoints([row(100000),row(200000)],0,5);
  assert.equal(chartSegments(close,p=>p.row.q_deg,false,[150000]).length,2);
});

test('invalid feedback and unlocked samples have no projected output; command validity is preserved',()=>{
  const rows=[row(100000,0,{feedback_valid:0}),row(200000,0,{locked:0}),row(300000,0,{faults:1})];
  assert.ok(directionPoints(rows,0,5).every(p=>p.projection===null&&!p.opposed));
  const points=directionPoints([row(100000),row(200000,0,{command_valid:1,final_ff_command_nm:2})],0,5);
  const segments=chartSegments(points,p=>p.row.command_valid?p.row.final_ff_command_nm:null);
  assert.equal(segments.length,1); assert.equal(segments[0].length,1);
  assert.equal(segments[0][0].row.timestamp_us,200000);
});

test('angle secants distinguish support, disagreement and a turning bracket without crossing gaps',()=>{
  const rows=orderedChartRows([row(100000,0,{q_deg:1}),row(200000,0,{q_deg:4,v_deg_s:20}),row(300000,0,{q_deg:3})]);
  const middle=directionPoints(rows,0,5)[1];
  assert.equal(middle.secant,10); assert.equal(middle.positionSupport,'supported'); assert.equal(middle.crossesTurn,true);
  rows[2]={...rows[2],q_deg:-2};
  assert.equal(directionPoints(rows,0,5)[1].positionSupport,'disagrees');
  rows[2]={...rows[2],timestamp_us:500000};
  assert.equal(directionPoints(rows,0,5)[1].secant,null);
});
