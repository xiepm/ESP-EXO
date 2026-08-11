import assert from "node:assert/strict";
import test from "node:test";

import {
  LOG_BOTTOM_THRESHOLD_PX,
  isLogNearBottom,
} from "../lib/exo-log-scroll.ts";

test("日志位于底部或阈值范围内时继续跟随", () => {
  assert.equal(isLogNearBottom({
    scrollHeight: 1000,
    scrollTop: 700,
    clientHeight: 300,
  }), true);

  assert.equal(isLogNearBottom({
    scrollHeight: 1000,
    scrollTop: 700 - LOG_BOTTOM_THRESHOLD_PX,
    clientHeight: 300,
  }), true);
});

test("用户向上滚动超过阈值后暂停跟随", () => {
  assert.equal(isLogNearBottom({
    scrollHeight: 1000,
    scrollTop: 600,
    clientHeight: 300,
  }), false);
});

test("空日志和小于视口的日志视为位于底部", () => {
  assert.equal(isLogNearBottom({
    scrollHeight: 200,
    scrollTop: 0,
    clientHeight: 300,
  }), true);
});
