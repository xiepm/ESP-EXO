import assert from 'node:assert/strict';
import {createHash} from 'node:crypto';
import {readFile} from 'node:fs/promises';
import test from 'node:test';
import {adaptiveDefaults, adaptiveErrors, parseAdaptiveConfig, encodeAdaptiveConfig,
  adaptiveCommands, gaussianPreview, parseAdaptiveLog, ADAPTIVE_COLUMNS, supportsAdaptive,adaptiveTrialPreset} from '../lib/exo-adaptive.ts';
import {AssistMode, AssistStrategy, AssistExecutionState, AssistUnavailableReason,
  assistUnavailableReasonDescriptions, Capability, encodeAssistConfigPayload,
  encodeAssistConfigWithLimitPayload, supportsAdaptiveBenchActive,adaptiveNominalMaximumNm,assistTorqueMaximumNm,encodeRequest,Opcode,parseResponse} from '../lib/exo-ble-protocol.ts';
import {assistMaximumDraftMatches, verifyAssistMaximumBeforeStart,
  assistMaximumDraftForStrategy, afoMotionStatus} from '../lib/exo-assist-settings.ts';

function afoReadback({left=50,right=50,reason=0,requested=2,execution=2,active=true,output=false}={}) {
  // The existing 16-byte BLE assist payload, including the new reason codes.
  return parseResponse(Uint8Array.of(1,0x87,1,16,1,5,active?5:0,
    requested|(execution<<4),5,7,0,left,right,1|(active?2:0)|(output?64:0),reason,1,0,0,3,0));
}
const afoLimit = {selectedValid:true,selectedMaximumNm:.5,activeValid:true,activeMaximumNm:.5};

test('ordinary AFO note shows saved zero levels and IDLE waiting for START, not an acquisition failure',()=>{
  const config=afoReadback({left:0,right:0,active:false,execution:0,reason:6});
  const status=afoMotionStatus(config,afoLimit,1);
  assert.match(status.label,/等待 START/);
  assert.match(status.description,/左侧已保存等级为0%/);
  assert.match(status.description,/右侧已保存等级为0%/);
  assert.match(status.description,/保存不会自动启动/);
  assert.doesNotMatch(status.description,/故障|捕获/);
  assert.equal(config.leftLevelPercent,0);assert.equal(config.rightLevelPercent,0);
});

test('AFO zero maximum and bilateral zero levels are explicit, while one disabled side is identified separately',()=>{
  assert.equal(afoMotionStatus(afoReadback(),{...afoLimit,activeMaximumNm:0},3).label,'零助力设置');
  assert.match(afoMotionStatus(afoReadback(),{...afoLimit,activeMaximumNm:0},3).description,/最大附加力矩为0/);
  assert.equal(afoMotionStatus(afoReadback({left:0,right:0}),afoLimit,3).label,'零助力设置');
  const partial=afoMotionStatus(afoReadback({left:0,output:true}),afoLimit,3);
  assert.equal(partial.label,'助力输出中');
  assert.match(partial.description,/左侧已保存等级为0%/);
  assert.doesNotMatch(partial.description,/右侧已保存等级为0%|双侧附加助力为零/);
});

test('ordinary AFO note displays reasons 9..11 from real-format responses, including partial output',()=>{
  for(const [reason,label] of [[9,'AFO正在捕获相位'],[10,'AFO保护已锁存'],[11,'AFO输入异常']]) {
    for(const output of [false,true]) {
      const config=afoReadback({reason,output});
      const status=afoMotionStatus(config,afoLimit,3);
      assert.equal(status.label,label);
      assert.ok(status.description.includes(assistUnavailableReasonDescriptions[reason]));
      assert.equal(status.description.includes('同时有非零附加助力已发送'),output);
      assert.doesNotMatch(status.description,/曲线过零/);
    }
  }
  assert.match(afoMotionStatus(afoReadback({reason:AssistUnavailableReason.AFO_REARM_REQUIRED}),afoLimit,3).description,/排除故障后需停止并重新启动/);
});

test('SHADOW and disabled execution never appear as live assistance or a curve zero crossing',()=>{
  const shadow=afoMotionStatus(afoReadback({requested:1,execution:1,reason:9}),afoLimit,3);
  assert.equal(shadow.label,'观察模式 · SHADOW');
  assert.match(shadow.description,/仅计算建议，不施加附加助力/);
  assert.ok(shadow.description.includes(assistUnavailableReasonDescriptions[9]));
  const disabled=afoMotionStatus(afoReadback({requested:0,execution:0}),afoLimit,3);
  assert.equal(disabled.label,'附加助力未启用');
  assert.doesNotMatch(disabled.description,/曲线过零/);
});

test('an ACTIVE zero sample is distinguished from mode disabled without asserting a proven zero crossing',()=>{
  const status=afoMotionStatus(afoReadback(),afoLimit,3);
  assert.equal(status.label,'当前附加助力为零');
  assert.match(status.description,/可能处于曲线过零，不等于助力模式关闭/);
  assert.equal(afoMotionStatus(afoReadback({output:true}),afoLimit,3).label,'助力输出中');
  assert.equal(afoMotionStatus(afoReadback(),{...afoLimit,selectedMaximumNm:0},3).label,'当前附加助力为零');
});

test('missing, non-AFO, transitional, unknown-reason and missing-limit readbacks cannot claim normal output',()=>{
  assert.equal(afoMotionStatus(null,afoLimit,1),null);
  assert.equal(afoMotionStatus({...afoReadback(),strategyId:1},afoLimit,1),null);
  for(const state of [undefined,0,2,4,5]) {
    const status=afoMotionStatus(afoReadback({output:true}),afoLimit,state);
    assert.notEqual(status.label,'助力输出中');
    assert.match(status.description,/不能确认AFO助力输出/);
  }
  assert.equal(afoMotionStatus(afoReadback({reason:99}),afoLimit,3).label,'AFO尚未就绪');
  const missing=afoMotionStatus(afoReadback(),{...afoLimit,activeValid:false},3);
  assert.equal(missing.label,'等待助力上限读回');
  assert.doesNotMatch(missing.description,/曲线过零/);
});

const savedMaximum = value => ({selectedValid:true,selectedMaximumNm:value});

test('maximum draft readiness rejects saved zero versus nonzero draft and the reverse',()=>{
  assert.equal(assistMaximumDraftMatches('0.500',savedMaximum(0)),false);
  assert.equal(assistMaximumDraftMatches('0',savedMaximum(.5)),false);
  assert.equal(assistMaximumDraftMatches('0',savedMaximum(5)),false);
  for(const value of [0,.5,3,5])
    assert.equal(assistMaximumDraftMatches(value.toFixed(3),savedMaximum(value)),true);
});

test('maximum readiness rejects invalid numeric input, missing readback and invalid saved state',()=>{
  for(const draft of ['', ' ', 'NaN', 'Infinity', '-Infinity', 'oops', '-0.5', '0.0001']) {
    assert.equal(assistMaximumDraftMatches(draft,savedMaximum(Number(draft))),false,draft);
  }
  for(const saved of [null,savedMaximum(null),{selectedValid:false,selectedMaximumNm:.5}])
    assert.equal(assistMaximumDraftMatches('0.500',saved),false);
});

test('fresh START verification rejects a changed device limit even when cached readiness matched',async()=>{
  for(const [draft,cached,fresh] of [['0.500',.5,0],['0',0,.5],['0',0,5]]) {
    assert.equal(assistMaximumDraftMatches(draft,savedMaximum(cached)),true);
    let reads=0;
    await assert.rejects(verifyAssistMaximumBeforeStart(draft,async()=>{
      reads++;return savedMaximum(fresh);
    }),/不能按旧上限START/);
    assert.equal(reads,1);
  }
});

test('START compares click-time intent even if earlier or current GET responses overwrite the form',async()=>{
  for(const [draft,saved] of [['0.500',0],['0',.5]]) {
    let formDraft=draft;
    const intendedMaximum=formDraft;
    // A previous GET can update the form before the final torque query starts.
    await Promise.resolve();formDraft=String(saved);
    await assert.rejects(verifyAssistMaximumBeforeStart(intendedMaximum,async()=>{
      await Promise.resolve();formDraft=String(saved);
      return savedMaximum(saved);
    }),/不能按旧上限START/);
    assert.equal(assistMaximumDraftMatches(formDraft,savedMaximum(saved)),true);
  }
});

test('fresh START verification accepts matching zero without enabling it and rejects invalid intent',async()=>{
  for(const value of [0,.5,3,5])
    await verifyAssistMaximumBeforeStart(value.toFixed(3),async()=>savedMaximum(value));
  for(const draft of ['', ' ', 'NaN', 'Infinity', '-1', '0.0001'])
    await assert.rejects(verifyAssistMaximumBeforeStart(draft,async()=>savedMaximum(Number(draft))));
  await assert.rejects(verifyAssistMaximumBeforeStart('.5',async()=>{throw new Error('read failed');}),/read failed/);
});

test('switching to regular AFO preserves 3 and 5 Nm; phase switching alone caps those values at .5',()=>{
  const identity=Object.freeze({firmware:[0,11,8],capabilities:0xff7f,
    strategyId:4,strategyVersion:4,offBodyBench:false});
  for(const draft of ['3','3.000','5','5.000']) {
    assert.equal(assistMaximumDraftForStrategy(draft,5,identity,15),draft);
    assert.equal(assistMaximumDraftForStrategy(draft,2,identity,15),'0.500');
  }
  assert.equal(assistMaximumDraftForStrategy('8',5,identity,15),'5.000');
  assert.equal(assistMaximumDraftForStrategy('5',5,identity,3),'3.000');
  assert.equal(assistMaximumDraftForStrategy('5',5,identity,null),'0.500');
  assert.equal(assistMaximumDraftForStrategy('15',4,identity,15),'15');
});

test('strategy switching preserves zero and invalid drafts and retains old AFO limits',()=>{
  const legacy={firmware:[0,11,1],capabilities:0xff7f,strategyId:5,strategyVersion:2,offBodyBench:true};
  assert.equal(assistMaximumDraftForStrategy('3',5,legacy,15),'0.500');
  for(const strategy of [2,4,5])
    for(const draft of ['0','0.000','',' ', 'NaN','Infinity','oops','-1'])
      assert.equal(assistMaximumDraftForStrategy(draft,strategy,legacy,15),draft);
});

test('0.11.19 recognizes strategy 9 and preserves the 15 Nm ordinary workflow',()=>{
  const identity={firmware:[0,11,19],capabilities:0xff7f,strategyId:5,strategyVersion:9,offBodyBench:false};
  assert.equal(supportsAdaptive(identity.firmware,identity.capabilities),true);
  assert.equal(supportsAdaptiveBenchActive(identity),true);
  assert.equal(supportsAdaptiveBenchActive({...identity,strategyVersion:8}),false);
  assert.equal(adaptiveNominalMaximumNm(identity),15);
});
test('0.11.20 recognizes strategy 10 without admitting unknown future strategy versions',()=>{
  const identity={firmware:[0,11,20],capabilities:0xff7f,strategyId:5,strategyVersion:10,offBodyBench:false};
  assert.equal(supportsAdaptive(identity.firmware,identity.capabilities),true);
  assert.equal(supportsAdaptiveBenchActive(identity),true);
  assert.equal(adaptiveNominalMaximumNm(identity),15);
  assert.equal(supportsAdaptiveBenchActive({...identity,strategyVersion:9}),false);
  assert.equal(supportsAdaptiveBenchActive({...identity,strategyVersion:11}),false);
  assert.equal(supportsAdaptiveBenchActive({...identity,firmware:[0,11,21]}),false);
});
test('0.11.21 recognizes walking strategy 11 with the existing 15 Nm workflow',()=>{
  const identity={firmware:[0,11,21],capabilities:0xff7f,strategyId:5,strategyVersion:11,offBodyBench:false};
  assert.equal(supportsAdaptive(identity.firmware,identity.capabilities),true);
  assert.equal(supportsAdaptiveBenchActive(identity),true);
  assert.equal(adaptiveNominalMaximumNm(identity),15);
  assert.equal(supportsAdaptiveBenchActive({...identity,strategyVersion:10}),false);
  assert.equal(supportsAdaptiveBenchActive({...identity,firmware:[0,11,24]}),false);
});
test('AFO default configuration equals delivery JSON and independently verified Python/C 196-byte golden',async()=>{
  const defaults=adaptiveDefaults();
  assert.deepEqual(defaults,JSON.parse(await readFile(new URL('../../config/adaptive_assist.json',import.meta.url),'utf8')));
  assert.deepEqual(adaptiveErrors(defaults),[]);
  const blob=encodeAdaptiveConfig(defaults);
  assert.equal(blob.length,196);
  assert.equal(createHash('sha256').update(blob).digest('hex'),'a5ddb4e92ab32436593483e16aeb8ad3a5b12007a092b2a01c60233d16ccad5c');
  defaults.left.strength=.5;assert.equal(defaults.right.strength,0);
});
test('AFO invalid or incomplete config cannot be imported or exported',()=>{
  for(const patch of [null,[],{}, {...adaptiveDefaults(),schema:2}, {...adaptiveDefaults(),extra:0},
    {...adaptiveDefaults(),overall_strength:'0'}, {...adaptiveDefaults(),overall_strength:NaN}]) {
    assert.ok(adaptiveErrors(patch).length);
    assert.throws(()=>parseAdaptiveConfig(JSON.stringify(patch)));
  }
  const c=adaptiveDefaults();c.afo.maximum_dt_s=.15;
  assert.throws(()=>encodeAdaptiveConfig(c));
  c.afo.maximum_dt_s=.03;c.left.enabled=1;
  assert.ok(adaptiveErrors(c).some(e=>e.includes('窗口')));
  c.left.enabled=.5;assert.ok(adaptiveErrors(c).length);
  c.left.enabled=0;c.afo.minimum_hz=2;c.afo.initial_hz=1;
  assert.ok(adaptiveErrors(c).length);
});
test('SHADOW selection retains old IDs and rejects strategy5 ACTIVE before encoding',()=>{
  const c={selectedMode:AssistMode.ADAPTIVE_GAUSSIAN,strategyId:AssistStrategy.ADAPTIVE_GAUSSIAN,
    requestedExecutionState:AssistExecutionState.SHADOW,profileId:0,leftLevelPercent:0,rightLevelPercent:0};
  assert.deepEqual([...encodeAssistConfigPayload(c)],[1,5,1,5,0,0,0,0]);
  assert.throws(()=>encodeAssistConfigPayload({...c,requestedExecutionState:AssistExecutionState.ACTIVE}),/ACTIVE/);
  assert.throws(()=>encodeAssistConfigWithLimitPayload({...c,requestedExecutionState:AssistExecutionState.ACTIVE},.5),/ACTIVE/);
  assert.throws(()=>encodeAssistConfigPayload({...c,profileId:1}));
  assert.doesNotThrow(()=>encodeAssistConfigPayload({...c,strategyId:2,selectedMode:2,profileId:1,requestedExecutionState:2}));
});
test('adaptive version guard requires actual known firmware and assist capability',()=>{
  assert.equal(supportsAdaptive([0,11,7],0xff7f),true);
  assert.equal(supportsAdaptive([0,11,8],0xff7f),true);
  assert.equal(supportsAdaptive([0,11,0],Capability.ASSIST_CONFIGURATION),true);
  assert.equal(supportsAdaptive([0,11,1],0xff7f),true);
  assert.equal(supportsAdaptive([0,11,2],0xff7f),true);
  assert.equal(supportsAdaptive([0,11,3],0xff7f),true);
  for(const firmware of [[],[0,10,13],[0,11,24]]) assert.equal(supportsAdaptive(firmware,0xff7f),false);
  assert.equal(supportsAdaptive([0,11,0],0xff7f & ~Capability.ASSIST_CONFIGURATION),false);
});

test('regular AFO mode selects ACTIVE directly from transparent, with the usual levels and limit',()=>{
  const identity={firmware:[0,11,7],capabilities:0xff7f,strategyId:1,strategyVersion:1,offBodyBench:false};
  const c={selectedMode:5,strategyId:5,requestedExecutionState:2,profileId:0,
    leftLevelPercent:25,rightLevelPercent:75,adaptiveBenchIdentity:identity};
  assert.equal(supportsAdaptiveBenchActive(identity),true);
  assert.equal(adaptiveNominalMaximumNm(identity),5);
  assert.deepEqual([...encodeAssistConfigWithLimitPayload(c,.5)],[1,5,2,5,0,25,75,0,244,1]);
  assert.doesNotThrow(()=>encodeAssistConfigWithLimitPayload(c,5));
  assert.throws(()=>encodeAssistConfigWithLimitPayload(c,5.001));
  assert.equal(supportsAdaptiveBenchActive({...identity,strategyId:5,strategyVersion:6}),true);
  assert.equal(supportsAdaptiveBenchActive({...identity,strategyId:5,strategyVersion:1}),false);
  assert.equal(supportsAdaptiveBenchActive({...identity,firmware:[0,11,0]}),false);
  assert.equal(supportsAdaptiveBenchActive({...identity,capabilities:0}),false);
});
test('walking 0.11.8 selects/saves the same payload, with version 7 readback and no START request',()=>{
  const identity={firmware:[0,11,8],capabilities:0xff7f,strategyId:1,strategyVersion:1,offBodyBench:false};
  const c={selectedMode:5,strategyId:5,requestedExecutionState:2,profileId:0,
    leftLevelPercent:0,rightLevelPercent:0,adaptiveBenchIdentity:identity};
  assert.equal(supportsAdaptiveBenchActive(identity),true);
  assert.equal(adaptiveNominalMaximumNm(identity),5);
  assert.deepEqual([...encodeAssistConfigWithLimitPayload(c,.5)],[1,5,2,5,0,0,0,0,244,1]);
  assert.equal(supportsAdaptiveBenchActive({...identity,strategyId:5,strategyVersion:7}),true);
  assert.equal(supportsAdaptiveBenchActive({...identity,strategyId:5,strategyVersion:6}),false);
  assert.equal(supportsAdaptiveBenchActive({...identity,firmware:[0,11,24]}),false);
  assert.equal(supportsAdaptiveBenchActive({...identity,capabilities:0}),false);
});

test('AFO bench ACTIVE requires current version, selected policy metadata and off-body mode; cannot encode 15Nm',()=>{
  const identity={firmware:[0,11,1],capabilities:0xff7f,strategyId:5,strategyVersion:2,offBodyBench:true};
  const config={selectedMode:5,strategyId:5,requestedExecutionState:2,profileId:0,leftLevelPercent:100,rightLevelPercent:100,
    adaptiveBenchIdentity:identity};
  assert.equal(supportsAdaptiveBenchActive(identity),true);
  assert.equal(supportsAdaptiveBenchActive({...identity,firmware:[0,11,2]}),true);
  assert.deepEqual([...encodeAssistConfigWithLimitPayload(config,.5)],[1,5,2,5,0,100,100,0,244,1]);
  assert.throws(()=>encodeAssistConfigWithLimitPayload(config,.501));
  assert.throws(()=>encodeAssistConfigWithLimitPayload(config,15));
  for(const patch of [{firmware:[0,11,0]},{firmware:[0,11,6]},{capabilities:0},{strategyId:1},{strategyVersion:1},{strategyVersion:3},{offBodyBench:false}]) {
    const bad={...identity,...patch};
    assert.equal(supportsAdaptiveBenchActive(bad),false);
    assert.throws(()=>encodeAssistConfigPayload({...config,adaptiveBenchIdentity:bad}),/ACTIVE/);
  }
});

test('0.11.4 keeps candidate and manual-release gates distinct',()=>{
  const candidate={firmware:[0,11,4],capabilities:0xff7f,strategyId:5,strategyVersion:3,offBodyBench:true};
  assert.equal(supportsAdaptive(candidate.firmware,candidate.capabilities),true);
  assert.equal(adaptiveNominalMaximumNm(candidate),15);
  assert.equal(supportsAdaptiveBenchActive(candidate),true);
  assert.equal(supportsAdaptiveBenchActive({...candidate,offBodyBench:false}),false);
  assert.equal(supportsAdaptiveBenchActive({...candidate,strategyVersion:4,offBodyBench:false}),true);
});

test('0.11.3 nominal candidate and manual release require distinct current strategy metadata',()=>{
  const candidate={firmware:[0,11,3],capabilities:0xff7f,strategyId:5,strategyVersion:3,offBodyBench:true};
  const values={selectedMode:5,strategyId:5,requestedExecutionState:2,profileId:0,leftLevelPercent:0,rightLevelPercent:0,adaptiveBenchIdentity:candidate};
  assert.equal(adaptiveNominalMaximumNm(candidate),15);
  const payload=encodeAssistConfigWithLimitPayload(values,15);
  assert.doesNotThrow(()=>encodeRequest(Opcode.SET_ASSIST_CONFIG_WITH_LIMIT,1,payload));
  assert.throws(()=>encodeAssistConfigWithLimitPayload(values,15.001));
  assert.throws(()=>encodeAssistConfigPayload({...values,adaptiveBenchIdentity:{...candidate,offBodyBench:false}}));
  assert.doesNotThrow(()=>encodeAssistConfigPayload({...values,adaptiveBenchIdentity:{...candidate,strategyVersion:4,offBodyBench:false}}));
  for(const patch of [{firmware:[0,11,2]},{strategyVersion:2},{capabilities:0},{strategyId:1}])
    assert.equal(adaptiveNominalMaximumNm({...candidate,...patch}),.5);
});
test('exported command transitions stay valid for frequency intervals above or below initial defaults',()=>{
  for(const [min,initial,max] of [[1.8,2,2.2],[.2,.3,.4]]) {
    const c=adaptiveDefaults();Object.assign(c.afo,{minimum_hz:min,initial_hz:initial,maximum_hz:max});
    const commands=adaptiveCommands(c).split('\n').filter(x=>x&&!x.startsWith('#'));
    const live=adaptiveDefaults();
    for(const command of commands) {
      assert.ok(!/^(start|stop|enable|joint_zero|set_zero|adaptive_shadow)\b/.test(command));
      const [name,group,key,raw]=command.split(' ');
      if(name==='adaptive_set') {
        if(group==='global')live[key]=Number(raw);else live[group][key]=Number(raw);
        assert.deepEqual(adaptiveErrors(live),[],command);
      }
    }
    assert.deepEqual(live,c);
    assert.ok(!commands.some(x=>x==='adaptive_store save'));
  }
});
test('periodic preview separates legs, respects disabled/zero and cancels overlapping peaks',()=>{
  const c=adaptiveDefaults();assert.equal(gaussianPreview(c,'left',.1),0);
  c.overall_strength=1;Object.assign(c.left,{enabled:1,strength:1,flex_amplitude_nm:.2,flex_peak_phase:.1});
  assert.ok(Math.abs(gaussianPreview(c,'left',.1)-.2)<1e-12);
  assert.equal(gaussianPreview(c,'right',.1),0);
  assert.ok(Math.abs(gaussianPreview(c,'left',0)-gaussianPreview(c,'left',1))<1e-12);
  c.left.ext_amplitude_nm=.2;c.left.ext_peak_phase=.1;
  assert.equal(gaussianPreview(c,'left',.1),0);
});
function row(timestamp=1000,side=0,patch={}) {
  const obj={timestamp_us:timestamp,side,period_s:2.5,...patch};
  return ADAPTIVE_COLUMNS.map(k=>obj[k]??0).join(',');
}
test('UART and CSV logs retain zero/invalid flags and detect malformed values or timestamp resets',()=>{
  const text=`boot\nADAPT,${row()}\r\nADAPT,${row(1000,1)}\nADAPT,${row(10)}\nADAPT,1,2,NaN\n`;
  const p=parseAdaptiveLog(text);assert.equal(p.rows.length,3);assert.equal(p.malformed,1);assert.equal(p.resets,1);
  assert.equal(p.rows[0].feedback_valid,0);assert.equal(p.rows[0].period_s,2.5);
  const csv=ADAPTIVE_COLUMNS.join(',')+'\n'+row(5,1,{feedback_valid:1,locked:1,confidence:.9})+'\n';
  assert.equal(parseAdaptiveLog(csv).rows[0].locked,1);
  for(const bad of [row(5,2),row(5,1,{locked:2}),row(5,1,{confidence:2}),row(5,1,{hz:Infinity}),row(5,1,{q_deg:''})])
    assert.equal(parseAdaptiveLog('ADAPT,'+bad).malformed,1);
});

test('0.11.5 trace candidate retains off-body policy and nominal cap',()=>{
 const c={firmware:[0,11,5],capabilities:0xff7f,strategyId:5,strategyVersion:3,offBodyBench:true};
 assert.equal(supportsAdaptive(c.firmware,c.capabilities),true);
 assert.equal(adaptiveNominalMaximumNm(c),15);
 assert.equal(supportsAdaptiveBenchActive(c),true);
 assert.equal(supportsAdaptiveBenchActive({...c,offBodyBench:false}),false);
});
test('0.11.6 bounded trial requires strategy5 and caps ACTIVE at 5 Nm',()=>{
 const identity={firmware:[0,11,6],capabilities:0xff7f,strategyId:5,strategyVersion:5,offBodyBench:false};
 assert.equal(supportsAdaptive(identity.firmware,identity.capabilities),true);
 assert.equal(supportsAdaptiveBenchActive(identity),true);
 assert.equal(adaptiveNominalMaximumNm(identity),5);
 for(const strategyVersion of [1,2,3,4,6])assert.equal(supportsAdaptiveBenchActive({...identity,strategyVersion}),false);
 const config={selectedMode:5,strategyId:5,requestedExecutionState:2,profileId:0,leftLevelPercent:100,rightLevelPercent:100,adaptiveBenchIdentity:identity};
 assert.equal(encodeAssistConfigWithLimitPayload(config,5).length,10);
 assert.throws(()=>encodeAssistConfigWithLimitPayload(config,5.001));
 for(const opcode of [Opcode.PREPARE_ADAPTIVE_TRIAL,Opcode.CHECK_ADAPTIVE_TRIAL]) {
  assert.equal(encodeRequest(opcode,12).length,4);
  assert.throws(()=>encodeRequest(opcode,12,new Uint8Array([1])));
  const ack=parseResponse(Uint8Array.of(1,opcode|0x80,12,4,0,1,0,1));
  assert.equal(ack.kind,"ack");assert.equal(ack.detail,1);
 }
 const preset=adaptiveTrialPreset();assert.deepEqual(adaptiveErrors(preset),[]);
 assert.equal(preset.left.negative_power_w,1);assert.equal(preset.left.torque_limit_nm,5);
});


test('0.11.17 RS01 regular AFO accepts15Nm and retains older firmware ceilings',()=>{
  const identity={firmware:[0,11,17],capabilities:0xff7f,strategyId:5,strategyVersion:8,offBodyBench:false};
  assert.equal(supportsAdaptiveBenchActive(identity),true);
  assert.equal(adaptiveNominalMaximumNm(identity),15);
  assert.equal(assistTorqueMaximumNm(5,identity.capabilities,15,identity.firmware),15);
  const values={selectedMode:5,strategyId:5,requestedExecutionState:2,profileId:0,leftLevelPercent:100,rightLevelPercent:100,adaptiveBenchIdentity:identity};
  const payload=encodeAssistConfigWithLimitPayload(values,15);
  assert.doesNotThrow(()=>encodeRequest(Opcode.SET_ASSIST_CONFIG_WITH_LIMIT,1,payload));
  assert.throws(()=>encodeAssistConfigWithLimitPayload(values,15.001));
  assert.equal(supportsAdaptiveBenchActive({...identity,strategyVersion:7}),false);
  for(const patch of [7,8,14,15,16]) {
    const older={...identity,firmware:[0,11,patch],strategyVersion:patch===7?6:7};
    assert.equal(adaptiveNominalMaximumNm(older),5);
    assert.equal(assistTorqueMaximumNm(5,identity.capabilities,15,older.firmware),5);
    assert.throws(()=>encodeAssistConfigWithLimitPayload({...values,adaptiveBenchIdentity:older},15));
  }
});

 test('0.11.22 accepts continuity strategy 12 and rejects mismatched versions',()=>{
  const identity={firmware:[0,11,22],capabilities:0xff7f,strategyId:5,strategyVersion:12,offBodyBench:false};
  assert.equal(supportsAdaptive(identity.firmware,identity.capabilities),true);
  assert.equal(supportsAdaptiveBenchActive(identity),true);
  assert.equal(adaptiveNominalMaximumNm(identity),15);
  assert.equal(supportsAdaptiveBenchActive({...identity,strategyVersion:11}),false);
  assert.equal(supportsAdaptiveBenchActive({...identity,firmware:[0,11,21]}),false);
  assert.equal(supportsAdaptiveBenchActive({...identity,firmware:[0,11,24]}),false);
});
