import assert from "node:assert/strict";
import test from "node:test";

import {
  assessDeviceCompatibility,
} from "../lib/exo-device-compatibility.ts";

function makeInfo(firmware = [0, 9, 0], overrides = {}) {
  return {
    protocolMinVersion: 1,
    protocolMaxVersion: 1,
    firmware,
    capabilities: 0x003f,
    maxFrameSize: 20,
    ...overrides,
  };
}

test("0.9.0 是首选固件且完全通过阶段 9 身份门", () => {
  const result = assessDeviceCompatibility(makeInfo());
  assert.equal(result.accepted, true);
  assert.deepEqual(result.blockingIssues, []);
  assert.deepEqual(result.warnings, []);
});

test("0.7.x 与 0.8.0 仍按各自能力门兼容，只给升级提示", () => {
  const result = assessDeviceCompatibility(makeInfo([0, 7, 0], {
    capabilities: 0x0007,
  }));
  assert.equal(result.accepted, true);
  assert.deepEqual(result.blockingIssues, []);
  assert.equal(result.warnings.length, 1);
  assert.match(result.warnings[0], /建议后续升级到 0\.9\.0/);

  const stage8 = assessDeviceCompatibility(makeInfo([0, 8, 0], {
    capabilities: 0x000f,
  }));
  assert.equal(stage8.accepted, true);
  assert.deepEqual(stage8.blockingIssues, []);
  assert.equal(stage8.warnings.length, 1);
});

test("未知固件、协议不匹配和能力缺失仍会阻止产品命令", () => {
  const result = assessDeviceCompatibility(makeInfo([1, 0, 0], {
    protocolMinVersion: 2,
    protocolMaxVersion: 2,
    capabilities: 0x0001,
  }));
  assert.equal(result.accepted, false);
  assert.equal(result.blockingIssues.length, 3);
  assert.match(result.blockingIssues.join("；"), /不在兼容范围/);
  assert.match(result.blockingIssues.join("；"), /协议范围/);
  assert.match(result.blockingIssues.join("；"), /缺少 START\/STOP/);
});

test("0.8.0 缺少阶段 8 capability 时阻止配置", () => {
  const result = assessDeviceCompatibility(makeInfo([0, 8, 0], {
    capabilities: 0x0007,
  }));
  assert.equal(result.accepted, false);
  assert.match(result.blockingIssues.join("；"), /控制配置能力/);
});

test("0.9.0 必须同时声明步态状态和助力配置 capability", () => {
  const noGait = assessDeviceCompatibility(makeInfo([0, 9, 0], {
    capabilities: 0x002f,
  }));
  assert.equal(noGait.accepted, false);
  assert.match(noGait.blockingIssues.join("；"), /步态状态能力/);

  const noAssist = assessDeviceCompatibility(makeInfo([0, 9, 0], {
    capabilities: 0x001f,
  }));
  assert.equal(noAssist.accepted, false);
  assert.match(noAssist.blockingIssues.join("；"), /助力策略配置能力/);
});

test("已知能力之外的新 capability 不会阻止 0.9.0", () => {
  const result = assessDeviceCompatibility(makeInfo([0, 9, 0], {
    capabilities: 0x803f,
  }));
  assert.equal(result.accepted, true);
  assert.deepEqual(result.blockingIssues, []);
});
