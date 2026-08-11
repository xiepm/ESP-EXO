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

test("server-renders the ESP-EXO hardware acceptance route", async () => {
  const response = await render();
  assert.equal(response.status, 200);
  assert.match(response.headers.get("content-type") ?? "", /^text\/html\b/i);

  const html = await response.text();
  assert.match(html, /<title>ESP-EXO 真机 BLE 验收台<\/title>/i);
  assert.match(html, /真机 BLE 验收台/);
  assert.match(html, /选择 ESP-EXO/);
  assert.match(html, /START 并等待 RUNNING/);
  assert.match(html, /STOP 并等待 IDLE/);
  assert.match(html, /模式、参考源与左右五元命令/);
  assert.match(html, /STOP → 应用全部 → 读回/);
  assert.match(html, /网页不会通过 BLE 连续发送五元命令/);
  assert.match(html, /步态估计/);
  assert.match(html, /助力模式与策略/);
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
  assert.match(page, /当前固件未声明 GAIT_STATUS capability/);
  assert.match(page, /当前固件未声明 ASSIST_CONFIGURATION capability/);
  assert.match(page, /GET_GAIT_STATUS/);
  assert.match(page, /GET_ASSIST_CONFIG/);
  assert.match(page, /SET_ASSIST_CONFIG/);
  assert.match(page, /SHADOW 只计算建议命令，不改变真实 CAN 输出/);
  assert.match(page, /左右周期\/相位不一致/);
  assert.match(page, /所需步态无效/);
  assert.match(page, /请求执行/);
  assert.match(page, /当前有效/);
  assert.match(page, /verified\.requestedExecutionState/);
  assert.match(page, /\[AssistExecutionState\.DISABLED, AssistExecutionState\.SHADOW\]/);
  assert.match(page, /queryGaitStatus\(\)/);
  assert.match(page, /await queryAssistConfig\(\)/);
  assert.doesNotMatch(page, /logEndRef.*scrollIntoView|scrollIntoView\(\{\s*block:\s*"end"/s);
  assert.doesNotMatch(page, /writeValueWithoutResponse|RobStride-CAN|6e400001/i);
  assert.match(startup, /http:\/\/localhost:4173\/exo/i);
});
