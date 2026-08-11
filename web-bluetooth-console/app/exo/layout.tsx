import type { Metadata } from "next";
import type { ReactNode } from "react";

const title = "ESP-EXO 真机 BLE 验收台";
const description =
  "通过 Product BLE v1 连接 ESP-EXO，完成双髋归零、START/STOP、状态轮询与阶段 7 数据导出。";

export const metadata: Metadata = {
  title,
  description,
  openGraph: {
    title,
    description,
    type: "website",
    images: [{ url: "/og.png", alt: title }],
  },
  twitter: {
    card: "summary_large_image",
    title,
    description,
    images: ["/og.png"],
  },
};

export default function ExoLayout({ children }: { children: ReactNode }) {
  return children;
}
