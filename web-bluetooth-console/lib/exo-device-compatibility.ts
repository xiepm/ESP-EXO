/**
 * BLE v1 保持同一帧头，以 capability 区分阶段能力：0.7.x 是核心功能，
 * 0.8.0 增加静态控制配置，0.9.0 再增加步态状态和助力策略配置。
 * 旧固件继续可用于其已经声明并验收过的功能。
 */
export const COMPATIBLE_FIRMWARE_VERSIONS = Object.freeze([
  "0.7.0",
  "0.7.1",
  "0.8.0",
  "0.9.0",
]);
export const PREFERRED_FIRMWARE_VERSION = "0.9.0";

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
  } else if (firmwareVersion !== PREFERRED_FIRMWARE_VERSION) {
    warnings.push(
      `固件 ${firmwareVersion} 兼容 BLE v1；建议后续升级到 ${PREFERRED_FIRMWARE_VERSION}`,
    );
  }
  if ((info.capabilities & REQUIRED_CORE_CAPABILITIES) !== REQUIRED_CORE_CAPABILITIES) {
    blockingIssues.push("缺少 START/STOP、软件零位或状态查询能力");
  }
  if ((firmwareVersion === "0.8.0" || firmwareVersion === "0.9.0") &&
      (info.capabilities & CONTROL_CONFIGURATION_CAPABILITY) === 0) {
    blockingIssues.push("阶段 8 固件未声明控制配置能力");
  }
  if (firmwareVersion === "0.9.0" &&
      (info.capabilities & GAIT_STATUS_CAPABILITY) === 0) {
    blockingIssues.push("阶段 9 固件未声明步态状态能力");
  }
  if (firmwareVersion === "0.9.0" &&
      (info.capabilities & ASSIST_CONFIGURATION_CAPABILITY) === 0) {
    blockingIssues.push("阶段 9 固件未声明助力策略配置能力");
  }
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
