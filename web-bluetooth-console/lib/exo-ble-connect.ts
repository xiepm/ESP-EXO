/**
 * Web Bluetooth 在 Windows 上偶尔会在第一次 GATT 建连时返回瞬时错误，
 * 即使设备仍在广播、第二次连接马上就能成功。这里把“只重试一次”的规则
 * 独立出来，避免页面各处自行循环，最终造成无限重连或多个并发连接。
 */
export type GattRetryOptions = {
  /** 两次连接之间的等待时间；产品页面固定使用 800 ms。 */
  delayMs?: number;
  /** 仅用于测试注入；正式页面默认使用 setTimeout。 */
  sleep?: (milliseconds: number) => Promise<void>;
  /** 第一次失败后、第二次尝试前，用于记录原始浏览器错误。 */
  onRetry?: (firstError: unknown) => void;
};

const defaultSleep = (milliseconds: number) =>
  new Promise<void>((resolve) => window.setTimeout(resolve, milliseconds));

/**
 * 执行一次 GATT 连接；若第一次失败，等待后使用同一设备再尝试一次。
 * 第二次仍失败时直接把第二次错误抛给调用者，绝不进行第三次尝试。
 */
export async function connectGattWithOneRetry<T>(
  connect: () => Promise<T>,
  options: GattRetryOptions = {},
): Promise<T> {
  const delayMs = options.delayMs ?? 800;
  const sleep = options.sleep ?? defaultSleep;

  try {
    return await connect();
  } catch (firstError) {
    options.onRetry?.(firstError);
    await sleep(delayMs);
    return connect();
  }
}
