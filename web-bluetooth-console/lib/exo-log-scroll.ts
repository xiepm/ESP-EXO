/** 日志容器距离底部小于该值时，认为操作者仍在查看最新日志。 */
export const LOG_BOTTOM_THRESHOLD_PX = 32;

export type LogScrollMetrics = {
  scrollHeight: number;
  scrollTop: number;
  clientHeight: number;
};

/**
 * 判断日志视口是否仍贴近底部。
 *
 * 用户一旦向上滚动超过阈值，页面就停止自动跟随；这样后续 BLE 日志
 * 仍可持续记录，但不会把操作者强行拉回最下面。
 */
export function isLogNearBottom(
  metrics: LogScrollMetrics,
  thresholdPx = LOG_BOTTOM_THRESHOLD_PX,
): boolean {
  const distanceFromBottom =
    metrics.scrollHeight - metrics.scrollTop - metrics.clientHeight;
  return distanceFromBottom <= Math.max(0, thresholdPx);
}
