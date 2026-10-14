import type { Metadata } from "next";
import type { ReactNode } from "react";

const title = "ESP-EXO 外骨骼上位机";
const description =
  "连接外骨骼、校准穿戴零位、设置助力等级，查看左右髋关节与按日期保存的步态记录。";

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
