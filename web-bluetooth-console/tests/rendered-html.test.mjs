import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";

async function render(pathname = "/exo") {
  const workerUrl = new URL("../dist/server/index.js", import.meta.url);
  workerUrl.searchParams.set("test", `${process.pid}-${Date.now()}-${Math.random()}`);
  const { default: worker } = await import(workerUrl.href);

  return worker.fetch(
    new Request(`http://localhost${pathname}`, {
      headers: { accept: "text/html" },
    }),
    {
      ASSETS: {
        fetch: async () => new Response("Not found", { status: 404 }),
      },
    },
    {
      waitUntil() {},
      passThroughOnException() {},
    },
  );
}

test("server-renders the five-section ESP-EXO console", async () => {
  const response = await render();
  assert.equal(response.status, 200);
  assert.match(response.headers.get("content-type") ?? "", /^text\/html\b/i);

  const html = await response.text();
  assert.match(html, /<title>ESP-EXO 外骨骼上位机<\/title>/i);
  assert.match(html, /上位机栏目/);
  for (const label of ["连接", "设置", "运动", "实时数据", "数据记录"]) assert.match(html, new RegExp(label));
  assert.match(html, /我的步态记录/);
  assert.match(html, /还没有步态记录/);
  assert.match(html, /开始观察记录/);
  assert.match(html, /选择 ESP-EXO/);
  assert.match(html, /穿戴后零位校准/);
  assert.match(html, /标定前抬腿为正/);
  assert.match(html, /<h2>出厂配置<\/h2>/);
  assert.match(html, /保存到主控NVS，断电后保留/);
  assert.match(html, /标定默认保留 · 需要时重新校准/);
  assert.match(html, /本次零位选择/);
  assert.match(html, /沿用上次零位/);
  assert.match(html, /重新校准/);
  assert.doesNotMatch(html, /第一步 · 每次穿戴后核对/);
  assert.ok(html.indexOf("前抬正方向标定") < html.indexOf("穿戴后零位校准"));
  const readableHtml = html.replace(/<!--[\s\S]*?-->/g, "");
  assert.match(readableHtml, /disabled=""[^>]*>记录左腿静止基准<\/button>/);
  assert.match(readableHtml, /disabled=""[^>]*>记录右腿静止基准<\/button>/);
  assert.match(html, /我已确认穿戴完成，双髋处于中立位并保持静止/);
  assert.match(html, /disabled=""[^>]*>检查校准条件（不写入零位）<\/button>/);
  assert.match(html, /已选择重新校准，请先检查下列条件/);
  assert.match(html, /选择此项不会采集零位/);
  assert.ok(html.indexOf("穿戴后零位校准") < html.indexOf("左右电机与标定读回"));
  assert.doesNotMatch(html, /本次校准已写入|双髋软件零位已写入并读回确认/);
  assert.match(html, /上电收到有效电机反馈后自动恢复/);
  assert.match(html, /已保存的零位在主控NVS/);
  assert.match(html, /START 并等待 RUNNING/);
  assert.match(html, /STOP 并等待 IDLE/);
  assert.match(html, /模式、参考源与左右五元命令/);
  assert.match(html, /STOP → 应用全部 → 读回/);
  assert.match(html, /网页不会通过 BLE 连续发送五元命令/);
  assert.match(html, /步态估计/);
  assert.match(html, /实时波形示波器/);
  assert.match(html, /波形时间窗口/);
  assert.match(html, /左右髋实时示波器/);
  assert.match(html, /等待实时数据/);
  assert.match(html, /助力模式与策略/);
  assert.match(html, /摆腿事件脉冲助力 · 单向前抬/);
  assert.match(html, /aria-label="助力策略"/);
  assert.match(html, /AFO双高斯助力/);
  assert.doesNotMatch(html, /AFO双高斯 · 仅SHADOW观察/);
  assert.match(html, /<details class="exo-advanced"><summary>AFO工程调试与日志（可选）<\/summary>/);
  assert.match(html, /AFO参数与日志工作台/);
  assert.match(html, /人工采集AFO · USB只读 · 可选5 N·m运行监测/);
  assert.match(html, /记录后续运行状态与±5 N·m指令/);
  assert.match(html, /AFO同时间轴方向对照图/);
  assert.match(html, /AFO相位与助力方向/);
  assert.match(html, /已有采集文件可以直接导入，无需重采/);
  assert.match(html, /仅图中比较幅值/);
  assert.match(html, /比较幅值只改变虚线，不写入设备/);
  assert.match(html, /等待本段AFO数据/);
  assert.match(html, /选择USB并开始只读采集/);
  assert.match(readableHtml, /disabled=""[^>]*>结束采集（不停止外骨骼）<\/button>/);
  assert.match(readableHtml, /disabled=""[^>]*>标记：开始活动<\/button>/);
  assert.match(html, /等待人工开始，尚未连接USB/);
  assert.match(html, /接入初期的启动日志会单独保留/);
  assert.match(html, /导出196字节配置/);
  assert.match(html, /导入AFO日志/);
  assert.match(html, /原HIP_EVENT_TBE髋角代理相位，不是AFO相位/);
  assert.match(html, /产品 ACTIVE 启动合同/);
  assert.match(html, /产品 ACTIVE 不下发位置步态轨迹/);
  assert.match(html, /本次 START 之后 LEFT、RIGHT 各自出现至少一个新的髋屈曲峰/);
  assert.match(html, /才会从 0 开始渐入助力力矩/);
  assert.match(html, /ALTERNATING_SINE_TEST 工程激励只能配合 SHADOW 验证/);
  assert.match(html, /工程验证激励/);
  assert.match(html, /隔离工程验收 · 非产品功能/);
  assert.match(html, /禁止 ACTIVE/);
  assert.match(html, /连接 0\.10\.1 工程固件后/);
  assert.match(html, /连接后将根据设备 capability 自动发现并读取步态状态/);
  assert.match(html, /连接后将根据设备 capability 自动发现并读取助力配置/);
  assert.match(html, /导出关节 CSV/);
  assert.doesNotMatch(html, /RobStride-CAN|6e400001-.*-9cca-00a0c9a2679e/i);
});

test("keeps the product route on binary Product BLE v1", async () => {
  const [page, protocol, startup] = await Promise.all([
    readFile(new URL("../app/exo/page.tsx", import.meta.url), "utf8"),
    readFile(new URL("../lib/exo-ble-protocol.ts", import.meta.url), "utf8"),
    readFile(new URL("../start-local.cmd", import.meta.url), "utf8"),
  ]);

  assert.match(protocol, /f7a10001-6b5a-4d92-9e7c-3a1c8b640001/i);
  assert.match(protocol, /f7a10002-6b5a-4d92-9e7c-3a1c8b640001/i);
  assert.match(protocol, /f7a10003-6b5a-4d92-9e7c-3a1c8b640001/i);
  assert.match(page, /startNotifications\(\)/);
  assert.match(page, /writeValueWithResponse/);
  assert.match(page, /encodeRequest\(/);
  assert.match(page, /parseResponse\(/);
  assert.match(page, /request_id/);
  assert.match(page, /重试已选设备/);
  assert.match(page, /已配对不等于已连接/);
  assert.match(page, /services:\s*\[EXO_BLE\.SERVICE_UUID\]/);
  assert.match(page, /connectGattWithOneRetry/);
  assert.match(page, /向上滚动会自动暂停跟随/);
  assert.match(page, /isLogNearBottom/);
  assert.match(page, /兼容固件提示/);
  assert.match(page, /等待左右电机主动反馈在线/);
  assert.match(page, /canConfirmNeutral/);
  assert.match(page, /Capability\.GAIT_STATUS/);
  assert.match(page, /Capability\.ASSIST_CONFIGURATION/);
  assert.match(page, /Capability\.ENGINEERING_TEST_STIMULUS/);
  assert.match(page, /当前固件未声明 GAIT_STATUS capability/);
  assert.match(page, /当前固件未声明 ASSIST_CONFIGURATION capability/);
  assert.match(page, /GET_GAIT_STATUS/);
  assert.match(page, /GET_ASSIST_CONFIG/);
  assert.match(page, /SET_ASSIST_CONFIG/);
  assert.match(page, /SHADOW 只计算建议命令，不改变真实 CAN 输出/);
  assert.match(page, /左右周期\/相位不一致/);
  assert.match(protocol, /所需步态无效/);
  assert.match(page, /请求执行/);
  assert.match(page, /当前有效/);
  assert.match(page, /verified\.requestedExecutionState/);
  assert.match(page, /AssistExecutionState\.ACTIVE/);
  assert.match(page, /ACTIVE 目前只允许叠加在 DAMPING \+ CONSTANT/);
  assert.match(page, /startBlockReasonDisplay/);
  assert.match(protocol, /控制模式\/参考源与助力执行状态不兼容/);
  assert.match(protocol, /RUN_CONTRACT_INVALID/);
  assert.match(protocol, /WAITING_NEW_GAIT_EVENTS/);
  assert.match(protocol, /等待本次 START 后左右髋各自出现新的屈曲峰/);
  assert.match(page, /ReferenceSource\.ALTERNATING_SINE_TEST/);
  assert.match(page, /一键准备 TEST \+ SHADOW/);
  assert.match(page, /TRANSPARENT \+ DISABLED/);
  assert.match(page, /工程激励只为打通/);
  assert.match(page, /ENGINEERING_TEST_JOINT_COMMAND_DRAFT/);
  assert.match(page, /PRODUCT_DAMPING_JOINT_COMMAND_DRAFT/);
  assert.match(page, /Kp=5\.0、Kd=0\.1/);
  assert.match(page, /START_TELEMETRY/);
  assert.match(page, /STOP_TELEMETRY/);
  assert.match(page, /Telemetry Notify/);
  assert.match(page, /暂停显示（继续采集）/);
  assert.match(page, /导出遥测 CSV/);
  const scope=await readFile(new URL("../lib/exo-scope.ts",import.meta.url),"utf8");
  assert.match(scope, /实际加入命令的助力增量/);
  assert.match(page, /实时波形示波器/);
  assert.match(page, /queryGaitStatus\(\)/);
  assert.match(page, /await queryAssistConfig\(\)/);
  assert.doesNotMatch(page, /logEndRef.*scrollIntoView|scrollIntoView\(\{\s*block:\s*"end"/s);
  assert.doesNotMatch(page, /writeValueWithoutResponse|RobStride-CAN|6e400001/i);
  assert.match(startup, /http:\/\/localhost:4173\/exo/i);
});
