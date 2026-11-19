import {
  AssistStrategy,
  AssistExecutionState,
  AssistUnavailableReason,
  assistUnavailableReasonDescriptions,
  adaptiveNominalMaximumNm,
  assistTorqueMaximumNm,
  type AdaptiveBenchIdentity,
  type AssistTorqueLimitResponse,
  type AssistConfigResponse,
} from "./exo-ble-protocol.ts";

type TorqueReadback = Pick<AssistTorqueLimitResponse, "selectedValid" | "selectedMaximumNm">;

export function assistMaximumDraftMatches(draft: string, saved: TorqueReadback | null): boolean {
  const value = Number(draft);
  // Match the encoder's 1 mN·m resolution; empty input must not become zero.
  return draft.trim() !== "" && Number.isFinite(value) && value >= 0 &&
    Math.abs(value * 1000 - Math.round(value * 1000)) <= 1e-8 &&
    saved?.selectedValid === true && saved.selectedMaximumNm === value;
}

/** Pass the click-time draft, before any readback can update page state. */
export async function verifyAssistMaximumBeforeStart(
  intendedDraft: string, readLimit: () => Promise<TorqueReadback>,
): Promise<void> {
  if (!assistMaximumDraftMatches(intendedDraft, await readLimit())) {
    throw new Error("最大附加助力草稿与设备已保存配置不同或无效；请先保存并读回，不能按旧上限START");
  }
}

export function assistMaximumForStrategy(strategyId: number,
  identity: AdaptiveBenchIdentity | undefined, reportedMaximumNm: number | null): number {
  if (strategyId === AssistStrategy.ADAPTIVE_GAUSSIAN) {
    return Math.min(adaptiveNominalMaximumNm(identity), reportedMaximumNm ?? .5);
  }
  if (!identity && strategyId === AssistStrategy.ANGLE_THRESHOLD) return 15;
  return assistTorqueMaximumNm(strategyId, identity?.capabilities ?? 0,
    reportedMaximumNm, identity?.firmware);
}

export function assistMaximumDraftForStrategy(draft: string, strategyId: number,
  identity: AdaptiveBenchIdentity | undefined, reportedMaximumNm: number | null): string {
  const maximum = assistMaximumForStrategy(strategyId, identity, reportedMaximumNm);
  const value = Number(draft);
  // Preserve zero and invalid drafts; validation handles invalid input explicitly.
  return Number.isFinite(value) && value > maximum ? maximum.toFixed(3) : draft;
}

/** Ordinary motion-panel status uses saved/active readback, never form drafts. */
export function afoMotionStatus(config: AssistConfigResponse | null,
  limit: AssistTorqueLimitResponse | null, systemState: number | undefined):
  { label: string; description: string } | null {
  if (!config?.decodedAssistFlags.selectedValid || config.strategyId !== AssistStrategy.ADAPTIVE_GAUSSIAN) return null;
  const running = systemState === 3;
  const active = config.decodedAssistFlags.activeValid;
  const maximum = running && active
    ? limit?.activeValid ? limit.activeMaximumNm : null
    : limit?.selectedValid ? limit.selectedMaximumNm : null;
  const notes: string[] = [];
  if (config.leftLevelPercent === 0) notes.push("左侧已保存等级为0%，该侧附加助力为零。");
  if (config.rightLevelPercent === 0) notes.push("右侧已保存等级为0%，该侧附加助力为零。");
  if (maximum === 0) notes.push("最大附加力矩为0 N·m，双侧附加助力为零。");
  if (maximum == null) notes.push("最大附加力矩尚未读回。");
  const status = (label: string, description: string) => ({label, description: [description, ...notes].join(" ")});
  const shadow = config.requestedExecutionState === AssistExecutionState.SHADOW ||
    (active && config.executionState === AssistExecutionState.SHADOW);
  if (systemState === 1) {
    return status("待机 · 等待 START", shadow
      ? "已保存AFO观察模式；START后仅计算建议，不施加附加助力。"
      : config.requestedExecutionState === AssistExecutionState.DISABLED
        ? "已保存AFO关闭设置；START也不会施加附加助力。"
        : "AFO设置已保存，等待START；保存不会自动启动。");
  }
  if (!running) {
    const label = systemState === 2 ? "启动中" : systemState === 4 ? "停止中"
      : systemState === 5 ? "系统故障" : "等待系统状态";
    return status(label, "尚未确认处于RUNNING，不能确认AFO助力输出。");
  }
  const reason = config.unavailableReason;
  const reasonDescription = assistUnavailableReasonDescriptions[reason];
  if (shadow) return status("观察模式 · SHADOW", `仅计算建议，不施加附加助力。${reasonDescription ?? ""}`);
  if (!active || config.executionState !== AssistExecutionState.ACTIVE) {
    return status("附加助力未启用", "当前运行未读回有效的AFO ACTIVE配置。");
  }
  if (reason !== AssistUnavailableReason.NONE) {
    const label = reason === AssistUnavailableReason.AFO_ACQUIRING ? "AFO正在捕获相位"
      : reason === AssistUnavailableReason.AFO_REARM_REQUIRED ? "AFO保护已锁存"
        : reason === AssistUnavailableReason.AFO_INPUT_FAULT ? "AFO输入异常" : "AFO尚未就绪";
    return status(label, `${reasonDescription ?? config.unavailableReasonName}。${
      config.decodedAssistFlags.realOutputEnabled ? "同时有非零附加助力已发送；请注意受影响侧的上述状态。" : ""}`);
  }
  if (maximum === 0 || (config.leftLevelPercent === 0 && config.rightLevelPercent === 0)) {
    return status("零助力设置", "当前设置不产生附加助力。");
  }
  if (config.decodedAssistFlags.realOutputEnabled) {
    return status("助力输出中", "设备报告已有非零附加助力发送。");
  }
  if (maximum == null) return status("等待助力上限读回", "尚不能完整核对本轮AFO配置。");
  return status("当前附加助力为零", "AFO ACTIVE已生效，当前未报告非零附加助力；可能处于曲线过零，不等于助力模式关闭。");
}
