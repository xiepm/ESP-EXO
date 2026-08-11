import assert from "node:assert/strict";
import test from "node:test";

import { connectGattWithOneRetry } from "../lib/exo-ble-connect.ts";

test("GATT 第一次成功时不等待也不重试", async () => {
  let connectCalls = 0;
  let sleepCalls = 0;

  const result = await connectGattWithOneRetry(
    async () => {
      connectCalls += 1;
      return "server";
    },
    {
      sleep: async () => { sleepCalls += 1; },
    },
  );

  assert.equal(result, "server");
  assert.equal(connectCalls, 1);
  assert.equal(sleepCalls, 0);
});

test("GATT 瞬时失败后等待 800 ms，并使用同一回调只重试一次", async () => {
  let connectCalls = 0;
  const waits = [];
  const retryErrors = [];

  const result = await connectGattWithOneRetry(
    async () => {
      connectCalls += 1;
      if (connectCalls === 1) throw new Error("first failure");
      return "server";
    },
    {
      sleep: async (milliseconds) => { waits.push(milliseconds); },
      onRetry: (error) => { retryErrors.push(error); },
    },
  );

  assert.equal(result, "server");
  assert.equal(connectCalls, 2);
  assert.deepEqual(waits, [800]);
  assert.equal(retryErrors.length, 1);
  assert.match(retryErrors[0].message, /first failure/);
});

test("GATT 连续失败两次后停止，绝不发起第三次连接", async () => {
  let connectCalls = 0;
  const waits = [];

  await assert.rejects(
    connectGattWithOneRetry(
      async () => {
        connectCalls += 1;
        throw new Error(`failure ${connectCalls}`);
      },
      {
        delayMs: 25,
        sleep: async (milliseconds) => { waits.push(milliseconds); },
      },
    ),
    /failure 2/,
  );

  assert.equal(connectCalls, 2);
  assert.deepEqual(waits, [25]);
});
