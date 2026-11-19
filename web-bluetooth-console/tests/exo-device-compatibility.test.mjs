import assert from "node:assert/strict";
import test from "node:test";

test("0.11.0开发固件保留既有协议能力，AFO不等于ACTIVE验收", () => {
  assert.equal(assessDeviceCompatibility(makeInfo([0, 11, 0], {capabilities: 0xff7f})).accepted, true);
});

test("0.10.6保持恢复能力门，完成后上电验证后不再提示待验候选", () => {
  const candidate = assessDeviceCompatibility(makeInfo([0, 10, 6], {capabilities: 0x0f7f}));
  assert.equal(candidate.accepted, true);
  assert.deepEqual(candidate.warnings, []);
  assert.equal(assessDeviceCompatibility(makeInfo([0, 10, 6], {capabilities: 0x077f})).accepted, false);
});

test("0.10.5 requires recovery diagnostics while 0.10.2 remains usable", () => {
  assert.equal(assessDeviceCompatibility(makeInfo([0, 10, 5], {capabilities: 0x0f7f})).accepted, true);
  const missing = assessDeviceCompatibility(makeInfo([0, 10, 5], {capabilities: 0x077f}));
  assert.equal(missing.accepted, false);
  assert.match(missing.blockingIssues.join("；"), /反馈恢复能力/);
  assert.equal(assessDeviceCompatibility(makeInfo([0, 10, 2], {capabilities: 0x017f})).accepted, true);
});

import {
  assessDeviceCompatibility,
} from "../lib/exo-device-compatibility.ts";

function makeInfo(firmware = [0, 10, 6], overrides = {}) {
  return {
    protocolMinVersion: 1,
    protocolMaxVersion: 1,
    firmware,
    capabilities: 0x0f7f,
    maxFrameSize: 20,
    ...overrides,
  };
}

test("0.10.6 是首选固件且声明方向/零位/力矩/恢复能力", () => {
  const result = assessDeviceCompatibility(makeInfo());
  assert.equal(result.accepted, true);
  assert.deepEqual(result.blockingIssues, []);
  assert.deepEqual(result.warnings, []);
});

test("0.10.2 缺少标定读回能力时拒绝身份", () => {
  const result = assessDeviceCompatibility(makeInfo([0, 10, 2], { capabilities: 0x007f }));
  assert.equal(result.accepted, false);
  assert.match(result.blockingIssues.join("；"), /零位与出厂映射/);
});

test("0.7.x 与 0.8.0 仍按各自能力门兼容，只给升级提示", () => {
  const result = assessDeviceCompatibility(makeInfo([0, 7, 0], {
    capabilities: 0x0007,
  }));
  assert.equal(result.accepted, true);
  assert.deepEqual(result.blockingIssues, []);
  assert.equal(result.warnings.length, 1);
  assert.match(result.warnings[0], /建议后续升级到 0\.10\.6/);

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

test("0.10.0 必须声明遥测流 capability", () => {
  const result = assessDeviceCompatibility(makeInfo([0, 10, 0], {
    capabilities: 0x003f,
  }));
  assert.equal(result.accepted, false);
  assert.match(result.blockingIssues.join("；"), /遥测流能力/);
});

test("0.10.0 保持兼容，0.10.1 的工程 capability 是可选能力而非产品身份门", () => {
  const previous = assessDeviceCompatibility(makeInfo([0, 10, 0], {
    capabilities: 0x007f,
  }));
  assert.equal(previous.accepted, true);
  assert.equal(previous.warnings.length, 1);

  const productOnly = assessDeviceCompatibility(makeInfo([0, 10, 1], {
    capabilities: 0x007f,
  }));
  assert.equal(productOnly.accepted, true);
  assert.equal(productOnly.warnings.length, 1);

  const engineering = assessDeviceCompatibility(makeInfo([0, 10, 1], {
    capabilities: 0x00ff,
  }));
  assert.equal(engineering.accepted, true);
  assert.deepEqual(engineering.blockingIssues, []);
});

test("0.10.3 requires torque-limit capability; 0.10.2 stays compatible", () => {
  assert.equal(assessDeviceCompatibility(makeInfo([0, 10, 3], {capabilities: 0x017f})).accepted, false);
  assert.equal(assessDeviceCompatibility(makeInfo([0, 10, 2], {capabilities: 0x017f})).accepted, true);
});

test("0.10.4 requires direction teaching; older read-only calibration stays compatible", () => {
  const missing = assessDeviceCompatibility(makeInfo([0, 10, 4], {capabilities: 0x037f}));
  assert.equal(missing.accepted, false);
  assert.match(missing.blockingIssues.join("；"), /前抬正方向标定/);
  assert.equal(assessDeviceCompatibility(makeInfo([0, 10, 3], {capabilities: 0x037f})).accepted, true);
});
