/**
 * BLE v1 保持同一帧头，以 capability 区分阶段能力：0.7.x 是核心功能，
 * 0.8.0 增加静态控制配置，0.9.0 增加步态与助力配置，0.10.0 增加遥测流，
 * 0.10.1 可选择性声明隔离工程验证激励。工程能力不是产品运行的身份门。
 * 旧固件继续可用于其已经声明并验收过的功能。
 */
export const COMPATIBLE_FIRMWARE_VERSIONS = Object.freeze([
  "0.7.0",
  "0.7.1",
  "0.8.0",
  "0.9.0",
  "0.10.0",
  "0.10.1",
  "0.10.2",
  "0.10.3",
  "0.10.4",
  "0.10.5",
  "0.10.6",
  "0.10.7",
  "0.10.8",
  "0.10.9",
  "0.10.10",
  "0.10.11",
  "0.10.12",
  "0.10.13",
  "0.11.0", "0.11.1", "0.11.2", "0.11.3", "0.11.4", "0.11.5", "0.11.6", "0.11.7", "0.11.8", "0.11.9", "0.11.10", "0.11.11", "0.11.12", "0.11.13", "0.11.14", "0.11.15", "0.11.16", "0.11.17", "0.11.18", "0.11.19", "0.11.20", "0.11.21", "0.11.22", "0.11.23",
]);
export const PREFERRED_FIRMWARE_VERSION = "0.10.6";

type ProductIdentity = {
  protocolMinVersion: number;
  protocolMaxVersion: number;
  firmware: readonly number[];
  capabilities: number;
  maxFrameSize: number;
};

// 与 BLE v1 协议常量保持一致。该文件保持无运行时依赖，便于 Node 原生测试
// 直接导入，不需要额外安装测试框架或 TypeScript loader。
const PROTOCOL_VERSION = 1;
const REQUIRED_CORE_CAPABILITIES = 0x0007;
const CONTROL_CONFIGURATION_CAPABILITY = 1 << 3;
const GAIT_STATUS_CAPABILITY = 1 << 4;
const ASSIST_CONFIGURATION_CAPABILITY = 1 << 5;
const TELEMETRY_STREAM_CAPABILITY = 1 << 6;
const CALIBRATION_STATUS_CAPABILITY = 1 << 8;
const MAX_FRAME_SIZE = 20;

export type DeviceCompatibility = {
  accepted: boolean;
  firmwareVersion: string | null;
  blockingIssues: string[];
  warnings: string[];
};

/**
 * 把“绝对不能执行命令”的协议问题与“建议升级”的版本提示分开。
 * 这避免已经通过真机验收的 0.7.0 被误锁，同时仍拒绝未知固件。
 */
export function assessDeviceCompatibility(
  info: ProductIdentity | null,
): DeviceCompatibility {
  if (!info) {
    return {
      accepted: false,
      firmwareVersion: null,
      blockingIssues: ["尚未读取 INFO"],
      warnings: [],
    };
  }

  const firmwareVersion = info.firmware.join(".");
  const recoveryFirmware = ["0.10.5", "0.10.6", "0.10.7", "0.10.8", "0.10.9", "0.10.10", "0.10.11", "0.10.12", "0.10.13", "0.11.0", "0.11.1", "0.11.2", "0.11.3", "0.11.4", "0.11.5", "0.11.6", "0.11.7", "0.11.8", "0.11.9", "0.11.10", "0.11.11", "0.11.12", "0.11.13", "0.11.14", "0.11.15", "0.11.16", "0.11.17", "0.11.18", "0.11.19", "0.11.20", "0.11.21", "0.11.22", "0.11.23"].includes(firmwareVersion);
  const directionFirmware = firmwareVersion === "0.10.4" || recoveryFirmware;
  const blockingIssues: string[] = [];
  const warnings: string[] = [];

  if (info.protocolMinVersion > PROTOCOL_VERSION ||
      info.protocolMaxVersion < PROTOCOL_VERSION) {
    blockingIssues.push(`协议范围不包含 v${PROTOCOL_VERSION}`);
  }
  if (!COMPATIBLE_FIRMWARE_VERSIONS.includes(firmwareVersion)) {
    blockingIssues.push(
      `固件 ${firmwareVersion} 不在兼容范围 ${COMPATIBLE_FIRMWARE_VERSIONS.join(" / ")}`,
    );
  } else if (firmwareVersion !== PREFERRED_FIRMWARE_VERSION && !["0.10.7", "0.10.8", "0.10.9", "0.10.10", "0.10.11", "0.10.12", "0.10.13", "0.11.0", "0.11.1", "0.11.2", "0.11.3", "0.11.4", "0.11.5", "0.11.6", "0.11.7", "0.11.8", "0.11.9", "0.11.10", "0.11.11", "0.11.12", "0.11.13", "0.11.14", "0.11.15", "0.11.16", "0.11.17", "0.11.18", "0.11.19", "0.11.20", "0.11.21", "0.11.22", "0.11.23"].includes(firmwareVersion)) {
    warnings.push(
      `固件 ${firmwareVersion} 兼容 BLE v1；建议后续升级到 ${PREFERRED_FIRMWARE_VERSION}`,
    );
  }
  if ((info.capabilities & REQUIRED_CORE_CAPABILITIES) !== REQUIRED_CORE_CAPABILITIES) {
    blockingIssues.push("缺少 START/STOP、软件零位或状态查询能力");
  }
  if ((directionFirmware || firmwareVersion === "0.8.0" || firmwareVersion === "0.9.0" ||
       firmwareVersion === "0.10.0" || firmwareVersion === "0.10.1" || (firmwareVersion === "0.10.2" || firmwareVersion === "0.10.3")) &&
      (info.capabilities & CONTROL_CONFIGURATION_CAPABILITY) === 0) {
    blockingIssues.push("阶段 8 固件未声明控制配置能力");
  }
  if ((directionFirmware || firmwareVersion === "0.9.0" || firmwareVersion === "0.10.0" ||
       firmwareVersion === "0.10.1" || (firmwareVersion === "0.10.2" || firmwareVersion === "0.10.3")) &&
      (info.capabilities & GAIT_STATUS_CAPABILITY) === 0) {
    blockingIssues.push("阶段 9 固件未声明步态状态能力");
  }
  if ((directionFirmware || firmwareVersion === "0.9.0" || firmwareVersion === "0.10.0" ||
       firmwareVersion === "0.10.1" || (firmwareVersion === "0.10.2" || firmwareVersion === "0.10.3")) &&
      (info.capabilities & ASSIST_CONFIGURATION_CAPABILITY) === 0) {
    blockingIssues.push("阶段 9 固件未声明助力策略配置能力");
  }
  if ((directionFirmware || firmwareVersion === "0.10.0" || firmwareVersion === "0.10.1" || (firmwareVersion === "0.10.2" || firmwareVersion === "0.10.3")) &&
      (info.capabilities & TELEMETRY_STREAM_CAPABILITY) === 0) {
    blockingIssues.push("阶段 10 固件未声明遥测流能力");
  }
  if ((directionFirmware || firmwareVersion === "0.10.2" || firmwareVersion === "0.10.3") &&
      (info.capabilities & CALIBRATION_STATUS_CAPABILITY) === 0) {
    blockingIssues.push(`${firmwareVersion} 未声明零位与出厂映射读回能力`);
  }
  if ((directionFirmware || firmwareVersion === "0.10.3") && (info.capabilities & (1 << 9)) === 0) {
    blockingIssues.push(`${firmwareVersion} 未声明最大附加助力力矩配置能力`);
  }
  if (directionFirmware && (info.capabilities & (1 << 10)) === 0) {
    blockingIssues.push(`${firmwareVersion} 未声明前抬正方向标定能力`);
  }
  if (recoveryFirmware && (info.capabilities & (1 << 11)) === 0)
    blockingIssues.push(`${firmwareVersion} 未声明CAN诊断与反馈恢复能力`);
  if (["0.10.7", "0.10.8", "0.10.9", "0.10.10", "0.10.11", "0.10.12", "0.10.13", "0.11.0", "0.11.1", "0.11.2", "0.11.3", "0.11.4", "0.11.5", "0.11.6", "0.11.7", "0.11.8", "0.11.9", "0.11.10", "0.11.11", "0.11.12", "0.11.13", "0.11.14", "0.11.15", "0.11.16", "0.11.17", "0.11.18", "0.11.19", "0.11.20", "0.11.21", "0.11.22", "0.11.23"].includes(firmwareVersion) && (info.capabilities & (1 << 12)) === 0)
    blockingIssues.push(`${firmwareVersion} 未声明角度阈值助力能力`);
  if (["0.10.8", "0.10.9", "0.10.10", "0.10.11", "0.10.12", "0.10.13", "0.11.0", "0.11.1", "0.11.2", "0.11.3", "0.11.4", "0.11.5", "0.11.6", "0.11.7", "0.11.8", "0.11.9", "0.11.10", "0.11.11", "0.11.12", "0.11.13", "0.11.14", "0.11.15", "0.11.16", "0.11.17", "0.11.18", "0.11.19", "0.11.20", "0.11.21", "0.11.22", "0.11.23"].includes(firmwareVersion) && (info.capabilities & (1 << 13)) === 0)
    blockingIssues.push(`${firmwareVersion} 未声明5 N·m角度助力能力`);
  if (["0.10.9", "0.10.10", "0.10.11", "0.10.12", "0.10.13", "0.11.0", "0.11.1", "0.11.2", "0.11.3", "0.11.4", "0.11.5", "0.11.6", "0.11.7", "0.11.8", "0.11.9", "0.11.10", "0.11.11", "0.11.12", "0.11.13", "0.11.14", "0.11.15", "0.11.16", "0.11.17", "0.11.18", "0.11.19", "0.11.20", "0.11.21", "0.11.22", "0.11.23"].includes(firmwareVersion) && (info.capabilities & (1 << 14)) === 0)
    blockingIssues.push("0.10.9 未声明可选沿用零位能力");
  if(["0.10.10", "0.10.11", "0.10.12", "0.10.13", "0.11.0", "0.11.1", "0.11.2", "0.11.3", "0.11.4", "0.11.5", "0.11.6", "0.11.7", "0.11.8", "0.11.9", "0.11.10", "0.11.11", "0.11.12", "0.11.13", "0.11.14", "0.11.15", "0.11.16", "0.11.17", "0.11.18", "0.11.19", "0.11.20", "0.11.21", "0.11.22", "0.11.23"].includes(firmwareVersion) && (info.capabilities & (1 << 15)) === 0)
    blockingIssues.push(`${firmwareVersion} 未声明单侧新摆腿事件脉冲能力`);
  if (info.maxFrameSize !== MAX_FRAME_SIZE) {
    blockingIssues.push(`最大帧长不是 ${MAX_FRAME_SIZE} B`);
  }

  return {
    accepted: blockingIssues.length === 0,
    firmwareVersion,
    blockingIssues,
    warnings,
  };
}
