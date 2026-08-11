/**
 * 本地 TypeScript 检查所需的最小 Cloudflare Worker 绑定声明。
 *
 * 真正运行时的对象由 Wrangler/Cloudflare 注入；网页 BLE 验收不使用 D1，
 * 这里只让 vinext 模板自带的 worker 与可选数据库入口能够参加严格类型检查。
 */
interface Fetcher {
  fetch(input: RequestInfo | URL, init?: RequestInit): Promise<Response>;
}

interface D1Database {
  readonly __d1DatabaseBrand?: never;
}

declare module "cloudflare:workers" {
  export const env: {
    DB?: D1Database;
  };
}
