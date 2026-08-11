#!/usr/bin/env python3
"""ESP-EXO 阶段 9 BLE 真机验收客户端。

这个脚本有两个明确的工作方式：

1. 默认方式只发送 GET_INFO、GET_GAIT_STATUS 和 GET_ASSIST_CONFIG，完全不修改
   设备状态，也不会发送 START、STOP 或归零命令；
2. 只有操作者显式添加 ``--verify-config`` 时，才会在确认整机处于 IDLE 后，
   临时写入 TRANSPARENT + SHADOW，验证读回和 ACTIVE 拒绝行为，最后恢复进入
   验收前的配置。

帧头、request_id 关联、BLE Notify transport 和测试报告器直接复用阶段 6 工具，
避免阶段 9 再实现一份容易逐渐不一致的基础协议代码。

常用命令：

    python tools/phase9_ble_acceptance.py --self-test
    python tools/phase9_ble_acceptance.py --name ESP-EXO
    python tools/phase9_ble_acceptance.py --name ESP-EXO --verify-config

``--self-test`` 是纯主机测试，不导入 bleak，也不需要连接硬件。
"""

from __future__ import annotations

import argparse
import asyncio
from dataclasses import dataclass
import struct
import sys
import time
from typing import Any, Callable, Sequence


# 既支持“python tools/xxx.py”直接执行，也支持测试代码把 tools 当命名空间包导入。
# 两条分支导入的是同一个阶段 6 实现，不在本文件复制 Frame/transport/Reporter。
try:  # pragma: no branch - 实际运行只会进入其中一个导入分支。
    from .phase6_ble_acceptance import (
        AcceptanceError,
        BleTransport,
        COMMAND_CHARACTERISTIC_UUID,
        DEFAULT_DEVICE_NAME,
        Frame,
        MAX_FRAME_SIZE,
        OP_GET_INFO,
        OP_GET_SYSTEM_STATUS,
        PROTOCOL_VERSION,
        RESPONSE_CHARACTERISTIC_UUID,
        RESULT_OK,
        RESULT_REJECTED_NOT_READY,
        Reporter,
        decode_frame,
        encode_request,
        expect_ack,
        parse_info,
        parse_system_status,
        require,
        require_correlated,
        resolve_device,
    )
except ImportError:  # 直接执行脚本时，脚本所在 tools 目录已经在 sys.path 中。
    from phase6_ble_acceptance import (  # type: ignore[no-redef]
        AcceptanceError,
        BleTransport,
        COMMAND_CHARACTERISTIC_UUID,
        DEFAULT_DEVICE_NAME,
        Frame,
        MAX_FRAME_SIZE,
        OP_GET_INFO,
        OP_GET_SYSTEM_STATUS,
        PROTOCOL_VERSION,
        RESPONSE_CHARACTERISTIC_UUID,
        RESULT_OK,
        RESULT_REJECTED_NOT_READY,
        Reporter,
        decode_frame,
        encode_request,
        expect_ack,
        parse_info,
        parse_system_status,
        require,
        require_correlated,
        resolve_device,
    )


# 阶段 9 新增操作码。响应操作码仍由阶段 6 的 require_correlated 按“请求 | 0x80”检查。
OP_GET_GAIT_STATUS = 0x06
OP_GET_ASSIST_CONFIG = 0x07
OP_SET_ASSIST_CONFIG = 0x16

# 本工具固定验收阶段 9 发布物，不提供“放宽版本”的命令行参数。这样烧错固件时会
# 立即失败，不会因为操作人员忘记填写 --expect-firmware 而得到假 PASS。
EXPECTED_FIRMWARE = (0, 9, 0)
EXPECTED_CAPABILITIES = 0x003F

# 固定线码来自 docs/phase6/BLE_PROTOCOL_V1.md；不要用内部 C enum 数值代替。
SYSTEM_STATE_IDLE = 1
ASSIST_SCHEMA_VERSION = 1
ASSIST_MODE_NONE = 0
ASSIST_MODE_TRANSPARENT = 1
ASSIST_MODE_ACTIVE_PHASE = 2
ASSIST_MODE_ASSIST_AS_NEEDED = 3
ASSIST_EXEC_DISABLED = 0
ASSIST_EXEC_SHADOW = 1
ASSIST_EXEC_ACTIVE = 2
ASSIST_STRATEGY_NONE = 0
ASSIST_STRATEGY_TRANSPARENT = 1
ASSIST_STRATEGY_PHASE_PROFILE = 2
ASSIST_STRATEGY_AAN_RESIDUAL = 3
ASSIST_UNAVAILABLE_ACTIVE_NOT_ALLOWED = 3

GAIT_FIXED_INVALID = 0xFFFF
GAIT_PHASE_MAX = 9999
GAIT_CADENCE_MAX = 30000
GAIT_CONFIDENCE_MAX = 10000
GAIT_FLAGS_MASK = 0x3F
GAIT_QUALITY_MASK = 0x7FFF

ASSIST_FLAG_SELECTED_VALID = 1 << 0
ASSIST_FLAG_ACTIVE_VALID = 1 << 1
ASSIST_FLAG_REAL_OUTPUT_ENABLED = 1 << 6
ASSIST_FLAGS_MASK = 0x7F

# 默认套件只允许三个 GET。这个常量也会进入 self-test，防止后续维护时把写命令
# 不小心塞进“默认只读”流程。
DEFAULT_READ_ONLY_OPCODES = (
    OP_GET_INFO,
    OP_GET_GAIT_STATUS,
    OP_GET_ASSIST_CONFIG,
)
MUTATING_OR_MOTION_OPCODES = {
    0x10,  # START
    0x11,  # STOP
    0x12,  # CAPTURE_ZERO
    0x13,  # CLEAR_ZERO
    0x14,  # SET_CONTROL_CONFIG
    0x15,  # SET_JOINT_COMMAND
    OP_SET_ASSIST_CONFIG,
}


@dataclass(frozen=True)
class GaitStatus:
    """GET_GAIT_STATUS 的 16-byte payload 解码结果。

    定点相位的 10000 表示 1.0，但合法线上值只到 9999；无效值必须是 0xFFFF。
    保留原始整数便于验收日志逐字节追溯，显示时再除以缩放系数。
    """

    estimator_id: int
    estimator_state: int
    gait_flags: int
    phase_origin: int
    quality_flags: int
    left_phase_x10000: int
    right_phase_x10000: int
    cadence_x100: int
    confidence_x10000: int
    estimate_age_ms: int

    @property
    def left_phase(self) -> float | None:
        return (
            None
            if self.left_phase_x10000 == GAIT_FIXED_INVALID
            else self.left_phase_x10000 / 10000.0
        )

    @property
    def right_phase(self) -> float | None:
        return (
            None
            if self.right_phase_x10000 == GAIT_FIXED_INVALID
            else self.right_phase_x10000 / 10000.0
        )

    @property
    def cadence_steps_per_minute(self) -> float | None:
        return (
            None
            if self.cadence_x100 == GAIT_FIXED_INVALID
            else self.cadence_x100 / 100.0
        )

    @property
    def confidence(self) -> float | None:
        return (
            None
            if self.confidence_x10000 == GAIT_FIXED_INVALID
            else self.confidence_x10000 / 10000.0
        )


@dataclass(frozen=True)
class AssistStatus:
    """GET_ASSIST_CONFIG 的稳定线格式，不是固件内部结构体内存布局。"""

    schema_version: int
    selected_mode: int
    active_mode: int
    requested_execution: int
    effective_execution: int
    strategy_id: int
    strategy_version: int
    profile_id: int
    left_level_percent: int
    right_level_percent: int
    assist_flags: int
    unavailable_reason: int
    config_generation: int
    required_sensor_mask: int
    available_sensor_mask: int

    @property
    def selected_valid(self) -> bool:
        return bool(self.assist_flags & ASSIST_FLAG_SELECTED_VALID)

    @property
    def active_valid(self) -> bool:
        return bool(self.assist_flags & ASSIST_FLAG_ACTIVE_VALID)

    @property
    def real_output_enabled(self) -> bool:
        return bool(self.assist_flags & ASSIST_FLAG_REAL_OUTPUT_ENABLED)


class RequestIds:
    """为一次 BLE 连接分配互不冲突的非零 request_id。"""

    def __init__(self) -> None:
        self._next = 1

    def take(self) -> int:
        require(1 <= self._next <= 0xFF, "request_id sequence exhausted")
        request_id = self._next
        self._next += 1
        return request_id


def parse_gait_status(frame: Frame) -> GaitStatus:
    """严格解析并检查 GAIT_STATUS 的哨兵、范围和 valid 位一致性。"""

    require(len(frame.payload) == 16, "GAIT_STATUS payload must be 16 bytes")
    payload = frame.payload
    require(payload[0] in (0, 1), "invalid gait estimator_id")
    require(payload[1] <= 4, "invalid gait estimator_state")
    require((payload[2] & ~GAIT_FLAGS_MASK) == 0, "unknown gait flag bits")
    require(payload[3] <= 2, "invalid gait phase_origin")

    quality, left, right, cadence, confidence, age = struct.unpack_from(
        "<HHHHHH", payload, 4
    )
    require((quality & ~GAIT_QUALITY_MASK) == 0, "unknown gait quality bits")

    left_valid = bool(payload[2] & (1 << 1))
    right_valid = bool(payload[2] & (1 << 2))
    overall_valid = bool(payload[2] & (1 << 0))
    any_phase_valid = left_valid or right_valid

    # valid 位为 0 时只能发送 0xFFFF，不能把“未知”伪装成相位 0；valid 位为 1
    # 时则必须落在 [0, 1) 的定点范围内。
    require(
        (left <= GAIT_PHASE_MAX) if left_valid else (left == GAIT_FIXED_INVALID),
        "left phase sentinel/range does not match valid flag",
    )
    require(
        (right <= GAIT_PHASE_MAX)
        if right_valid
        else (right == GAIT_FIXED_INVALID),
        "right phase sentinel/range does not match valid flag",
    )
    require(
        not overall_valid or (left_valid and right_valid),
        "overall gait valid requires both side phases",
    )
    require(
        (cadence <= GAIT_CADENCE_MAX)
        if any_phase_valid
        else (cadence == GAIT_FIXED_INVALID),
        "cadence sentinel/range does not match phase validity",
    )
    require(
        (confidence <= GAIT_CONFIDENCE_MAX)
        if any_phase_valid
        else (confidence == GAIT_FIXED_INVALID),
        "confidence sentinel/range does not match phase validity",
    )

    return GaitStatus(
        estimator_id=payload[0],
        estimator_state=payload[1],
        gait_flags=payload[2],
        phase_origin=payload[3],
        quality_flags=quality,
        left_phase_x10000=left,
        right_phase_x10000=right,
        cadence_x100=cadence,
        confidence_x10000=confidence,
        estimate_age_ms=age,
    )


def _mode_matches_strategy(mode: int, strategy: int) -> bool:
    return (mode, strategy) in {
        (ASSIST_MODE_TRANSPARENT, ASSIST_STRATEGY_TRANSPARENT),
        (ASSIST_MODE_ACTIVE_PHASE, ASSIST_STRATEGY_PHASE_PROFILE),
        (ASSIST_MODE_ASSIST_AS_NEEDED, ASSIST_STRATEGY_AAN_RESIDUAL),
    }


def parse_assist_status(frame: Frame) -> AssistStatus:
    """解析 ASSIST_CONFIG，尤其严格拆分第 4 个 payload 字节的两个半字节。"""

    require(len(frame.payload) == 16, "ASSIST_CONFIG payload must be 16 bytes")
    payload = frame.payload
    require(payload[0] == ASSIST_SCHEMA_VERSION, "assist schema must be 1")
    require(payload[1] <= ASSIST_MODE_ASSIST_AS_NEEDED, "invalid selected mode")
    require(payload[2] <= ASSIST_MODE_ASSIST_AS_NEEDED, "invalid active mode")

    # 这里不能把整个字节当作一个 enum：0x01 表示“请求 SHADOW、当前 DISABLED”，
    # 0x11 才表示“请求 SHADOW、当前也正在 SHADOW”。每个半字节 3..15 均非法。
    execution_states = payload[3]
    requested = execution_states & 0x0F
    effective = (execution_states >> 4) & 0x0F
    require(requested <= ASSIST_EXEC_ACTIVE, "invalid requested execution nibble")
    require(effective <= ASSIST_EXEC_ACTIVE, "invalid effective execution nibble")

    require(payload[4] <= ASSIST_STRATEGY_AAN_RESIDUAL, "invalid strategy_id")
    require(payload[7] <= 100, "left assist level exceeds 100 percent")
    require(payload[8] <= 100, "right assist level exceeds 100 percent")
    require((payload[9] & ~ASSIST_FLAGS_MASK) == 0, "unknown assist flag bits")
    require(payload[10] <= 7, "invalid unavailable_reason")

    selected_valid = bool(payload[9] & ASSIST_FLAG_SELECTED_VALID)
    active_valid = bool(payload[9] & ASSIST_FLAG_ACTIVE_VALID)
    real_output = bool(payload[9] & ASSIST_FLAG_REAL_OUTPUT_ENABLED)

    require(
        selected_valid
        or (
            payload[1] == ASSIST_MODE_NONE
            and payload[4] == ASSIST_STRATEGY_NONE
            and requested == ASSIST_EXEC_DISABLED
        ),
        "invalid values in an unselected assist slot",
    )
    require(
        not selected_valid or _mode_matches_strategy(payload[1], payload[4]),
        "selected assist mode does not match strategy",
    )
    require(
        active_valid or payload[2] == ASSIST_MODE_NONE,
        "inactive assist slot must use mode NONE",
    )
    require(
        not active_valid
        or ASSIST_MODE_TRANSPARENT
        <= payload[2]
        <= ASSIST_MODE_ASSIST_AS_NEEDED,
        "active assist slot must use a selectable mode",
    )
    require(
        active_valid or effective == ASSIST_EXEC_DISABLED,
        "inactive assist slot must be effectively DISABLED",
    )
    require(not active_valid or selected_valid, "active assist requires selected config")
    require(
        not real_output or effective == ASSIST_EXEC_ACTIVE,
        "real output flag requires effective ACTIVE",
    )

    required_mask, available_mask = struct.unpack_from("<HH", payload, 12)
    return AssistStatus(
        schema_version=payload[0],
        selected_mode=payload[1],
        active_mode=payload[2],
        requested_execution=requested,
        effective_execution=effective,
        strategy_id=payload[4],
        strategy_version=payload[5],
        profile_id=payload[6],
        left_level_percent=payload[7],
        right_level_percent=payload[8],
        assist_flags=payload[9],
        unavailable_reason=payload[10],
        config_generation=payload[11],
        required_sensor_mask=required_mask,
        available_sensor_mask=available_mask,
    )


def make_assist_set_payload(
    *,
    mode: int,
    execution: int,
    strategy: int,
    profile_id: int,
    left_level_percent: int,
    right_level_percent: int,
) -> bytes:
    """按 8-byte SET_ASSIST_CONFIG 线格式构造请求 payload。"""

    require(_mode_matches_strategy(mode, strategy), "assist mode/strategy mismatch")
    require(
        execution in (ASSIST_EXEC_DISABLED, ASSIST_EXEC_SHADOW, ASSIST_EXEC_ACTIVE),
        "invalid requested assist execution",
    )
    require(0 <= profile_id <= 0xFF, "profile_id must fit u8")
    require(0 <= left_level_percent <= 100, "left level must be 0..100")
    require(0 <= right_level_percent <= 100, "right level must be 0..100")
    # 最后一个 reserved_flags 在阶段 9 必须固定为 0，不能拿它携带本地私有开关。
    return bytes(
        (
            ASSIST_SCHEMA_VERSION,
            mode,
            execution,
            strategy,
            profile_id,
            left_level_percent,
            right_level_percent,
            0,
        )
    )


def restorable_payload(status: AssistStatus) -> bytes:
    """把 GET 快照转换回 SET payload；仅接受阶段 9 能可靠恢复的 selected 配置。"""

    require(status.selected_valid, "original assist config has no selected slot")
    require(not status.active_valid, "original assist config is active; IDLE required")
    require(
        status.effective_execution == ASSIST_EXEC_DISABLED,
        "original assist config is effectively running",
    )
    require(
        status.requested_execution in (ASSIST_EXEC_DISABLED, ASSIST_EXEC_SHADOW),
        "original ACTIVE request cannot be restored by stage 9 firmware",
    )
    return make_assist_set_payload(
        mode=status.selected_mode,
        execution=status.requested_execution,
        strategy=status.strategy_id,
        profile_id=status.profile_id,
        left_level_percent=status.left_level_percent,
        right_level_percent=status.right_level_percent,
    )


async def _get_gait(transport: BleTransport, request_id: int) -> GaitStatus:
    raw = await transport.transact(encode_request(OP_GET_GAIT_STATUS, request_id))
    frame = decode_frame(raw)
    require_correlated(frame, OP_GET_GAIT_STATUS, request_id)
    return parse_gait_status(frame)


async def _get_assist(transport: BleTransport, request_id: int) -> AssistStatus:
    raw = await transport.transact(encode_request(OP_GET_ASSIST_CONFIG, request_id))
    frame = decode_frame(raw)
    require_correlated(frame, OP_GET_ASSIST_CONFIG, request_id)
    return parse_assist_status(frame)


async def run_read_only_tests(
    transport: BleTransport, reporter: Reporter, ids: RequestIds
) -> AssistStatus:
    """执行默认三项 GET；此函数没有任何 SET 调用。"""

    request_id = ids.take()
    raw = await transport.transact(encode_request(OP_GET_INFO, request_id))
    frame = decode_frame(raw)
    require_correlated(frame, OP_GET_INFO, request_id)
    info = parse_info(frame)
    require(
        info.protocol_min == PROTOCOL_VERSION
        and info.protocol_max == PROTOCOL_VERSION,
        "device protocol range is not exactly v1",
    )
    require(
        info.firmware == EXPECTED_FIRMWARE,
        "firmware mismatch: got "
        f"{'.'.join(map(str, info.firmware))}, expected 0.9.0",
    )
    require(
        info.capabilities == EXPECTED_CAPABILITIES,
        f"capabilities=0x{info.capabilities:04X}, expected 0x003F",
    )
    require(info.max_frame_size == MAX_FRAME_SIZE, "device max frame is not 20")
    reporter.passed_test(
        "GET_INFO strict stage 9 identity",
        f"fw=0.9.0 caps=0x003F boot_id=0x{info.boot_id:08X}",
    )

    gait = await _get_gait(transport, ids.take())
    reporter.passed_test(
        "GET_GAIT_STATUS",
        f"estimator={gait.estimator_id} state={gait.estimator_state} "
        f"flags=0x{gait.gait_flags:02X} quality=0x{gait.quality_flags:04X} "
        f"left={gait.left_phase} right={gait.right_phase} "
        f"cadence={gait.cadence_steps_per_minute} "
        f"confidence={gait.confidence} age={gait.estimate_age_ms}ms",
    )

    assist = await _get_assist(transport, ids.take())
    # 阶段 9 无论是否 START，真实输出位都必须保持为 0；SHADOW 只能生成诊断建议。
    require(not assist.real_output_enabled, "stage 9 unexpectedly enables real output")
    reporter.passed_test(
        "GET_ASSIST_CONFIG",
        f"selected={assist.selected_mode} active={assist.active_mode} "
        f"requested={assist.requested_execution} "
        f"effective={assist.effective_execution} flags=0x{assist.assist_flags:02X} "
        f"reason={assist.unavailable_reason} generation={assist.config_generation}",
    )
    return assist


async def _set_assist(
    transport: BleTransport, request_id: int, payload: bytes
) -> None:
    raw = await transport.transact(
        encode_request(OP_SET_ASSIST_CONFIG, request_id, payload)
    )
    expect_ack(raw, OP_SET_ASSIST_CONFIG, request_id, RESULT_OK)


async def run_config_verification(
    transport: BleTransport,
    reporter: Reporter,
    ids: RequestIds,
    original: AssistStatus,
) -> None:
    """在 IDLE 临时验收 SHADOW/ACTIVE 行为，并保证退出前恢复 selected 配置。"""

    # SET 之前先读整机状态。不能只看 ASSIST active_valid，因为维护门判定的权威来源
    # 是 SYSTEM_STATUS；RUNNING/STARTING/STOPPING 时本工具绝不尝试“碰碰运气”写配置。
    system_id = ids.take()
    system_raw = await transport.transact(
        encode_request(OP_GET_SYSTEM_STATUS, system_id)
    )
    system_frame = decode_frame(system_raw)
    require_correlated(system_frame, OP_GET_SYSTEM_STATUS, system_id)
    system = parse_system_status(system_frame)
    require(
        system.state == SYSTEM_STATE_IDLE,
        f"--verify-config requires IDLE, current state={system.state}",
    )
    original_payload = restorable_payload(original)
    reporter.passed_test("configuration precondition", "system=IDLE; original is restorable")

    shadow_payload = make_assist_set_payload(
        mode=ASSIST_MODE_TRANSPARENT,
        execution=ASSIST_EXEC_SHADOW,
        strategy=ASSIST_STRATEGY_TRANSPARENT,
        profile_id=0,
        left_level_percent=0,
        right_level_percent=0,
    )
    active_payload = make_assist_set_payload(
        mode=ASSIST_MODE_TRANSPARENT,
        execution=ASSIST_EXEC_ACTIVE,
        strategy=ASSIST_STRATEGY_TRANSPARENT,
        profile_id=0,
        left_level_percent=0,
        right_level_percent=0,
    )

    mutation_attempted = False
    verification_error: BaseException | None = None
    try:
        # 在发送前就置位：即使设备已执行 SET、但 Notify 恰好超时，finally 仍会尝试恢复。
        mutation_attempted = True
        await _set_assist(transport, ids.take(), shadow_payload)
        shadow = await _get_assist(transport, ids.take())
        require(restorable_payload(shadow) == shadow_payload, "SHADOW readback mismatch")
        require(
            shadow.requested_execution == ASSIST_EXEC_SHADOW,
            "SHADOW requested nibble was not preserved",
        )
        require(
            shadow.effective_execution == ASSIST_EXEC_DISABLED,
            "IDLE SHADOW must be effectively DISABLED",
        )
        require(not shadow.active_valid, "IDLE SHADOW unexpectedly became active")
        require(not shadow.real_output_enabled, "SHADOW enabled real output")
        reporter.passed_test(
            "SET/GET TRANSPARENT + SHADOW",
            "execution_states=0x01 (requested=SHADOW, effective=DISABLED)",
        )

        # ACTIVE 在格式层是可识别值，但阶段 9 业务层必须明确拒绝为 0x10/detail=3。
        active_id = ids.take()
        active_raw = await transport.transact(
            encode_request(OP_SET_ASSIST_CONFIG, active_id, active_payload)
        )
        active_ack = expect_ack(
            active_raw,
            OP_SET_ASSIST_CONFIG,
            active_id,
            RESULT_REJECTED_NOT_READY,
        )
        require(
            active_ack.detail == ASSIST_UNAVAILABLE_ACTIVE_NOT_ALLOWED,
            f"ACTIVE rejection detail={active_ack.detail}, expected 3",
        )
        after_active = await _get_assist(transport, ids.take())
        require(
            restorable_payload(after_active) == shadow_payload,
            "rejected ACTIVE changed the selected configuration",
        )
        require(
            after_active.config_generation == shadow.config_generation,
            "rejected ACTIVE changed config_generation",
        )
        # ACTIVE_NOT_ALLOWED 属于“本次请求为何被拒绝”，权威证据是上面的 ACK detail。
        # 被拒绝的请求不能改写 selected 快照，所以后续 GET 仍应描述原 SHADOW 配置
        # 当前为何未生效（例如 IDLE 下常见的 NOT_ARMED），不能要求它保存一次性的
        # 拒绝原因。这里专门检查配置和 generation 不变，防止验收脚本反过来要求
        # 固件污染持久状态。
        require(not after_active.real_output_enabled, "ACTIVE rejection enabled output")
        reporter.passed_test(
            "ACTIVE is rejected without config change",
            "ACK result=0x10 detail=3; selected remains SHADOW",
        )
    except BaseException as error:
        verification_error = error
    finally:
        if mutation_attempted:
            try:
                await _set_assist(transport, ids.take(), original_payload)
                restored = await _get_assist(transport, ids.take())
                require(
                    restorable_payload(restored) == original_payload,
                    "restored selected configuration does not match original",
                )
                require(
                    not restored.real_output_enabled,
                    "restoration unexpectedly enabled real output",
                )
                reporter.passed_test(
                    "restore original assist configuration",
                    "all SET-addressable fields match pre-test snapshot",
                )
            except BaseException as restore_error:
                if verification_error is not None:
                    raise AcceptanceError(
                        "configuration verification failed and restoration also failed: "
                        f"verify={verification_error}; restore={restore_error}"
                    ) from restore_error
                raise

    if verification_error is not None:
        raise verification_error


def _expect_parse_failure(function: Callable[[], object], message: str) -> None:
    try:
        function()
    except AcceptanceError:
        return
    raise AcceptanceError(message)


def run_self_test(reporter: Reporter) -> None:
    """不连接 BLE 的 golden-vector 与边界回归。"""

    checks: list[tuple[str, Callable[[], None]]] = []

    def check(name: str) -> Callable[[Callable[[], None]], Callable[[], None]]:
        def decorate(function: Callable[[], None]) -> Callable[[], None]:
            checks.append((name, function))
            return function

        return decorate

    @check("default plan is exactly three read-only GETs")
    def _default_plan_is_read_only() -> None:
        require(
            DEFAULT_READ_ONLY_OPCODES
            == (OP_GET_INFO, OP_GET_GAIT_STATUS, OP_GET_ASSIST_CONFIG),
            "default opcode list changed",
        )
        require(
            MUTATING_OR_MOTION_OPCODES.isdisjoint(DEFAULT_READ_ONLY_OPCODES),
            "mutating/motion opcode leaked into default plan",
        )

    @check("golden requests: GET_GAIT and GET_ASSIST")
    def _golden_get_requests() -> None:
        require(
            encode_request(OP_GET_GAIT_STATUS, 0x21)
            == bytes.fromhex("01 06 21 00"),
            "GET_GAIT request mismatch",
        )
        require(
            encode_request(OP_GET_ASSIST_CONFIG, 0x22)
            == bytes.fromhex("01 07 22 00"),
            "GET_ASSIST request mismatch",
        )

    @check("golden response: invalid gait uses 0xFFFF sentinels")
    def _golden_invalid_gait() -> None:
        frame = decode_frame(
            bytes.fromhex(
                "01 86 21 10 01 04 00 01 03 00 FF FF FF FF FF FF FF FF 05 00"
            )
        )
        require_correlated(frame, OP_GET_GAIT_STATUS, 0x21)
        gait = parse_gait_status(frame)
        require(gait.left_phase is None, "left invalid sentinel was displayed")
        require(gait.right_phase is None, "right invalid sentinel was displayed")
        require(gait.cadence_steps_per_minute is None, "invalid cadence displayed")
        require(gait.confidence is None, "invalid confidence displayed")

    @check("golden response: valid gait fixed-point decode")
    def _golden_valid_gait() -> None:
        frame = decode_frame(
            bytes.fromhex(
                "01 86 22 10 01 03 0F 01 00 00 C4 09 4C 1D 10 27 40 1F 0A 00"
            )
        )
        require_correlated(frame, OP_GET_GAIT_STATUS, 0x22)
        gait = parse_gait_status(frame)
        require(gait.left_phase == 0.25, "left phase decode mismatch")
        require(gait.right_phase == 0.75, "right phase decode mismatch")
        require(gait.cadence_steps_per_minute == 100.0, "cadence mismatch")
        require(gait.confidence == 0.8, "confidence mismatch")

    def assist_frame(execution_states: int, *, active: bool = False) -> Frame:
        active_mode = 1 if active else 0
        flags = 0x0B if active else 0x09
        raw = bytes(
            (
                1,
                0x87,
                0x31,
                16,
                1,
                1,
                active_mode,
                execution_states,
                1,
                1,
                0,
                0,
                0,
                flags,
                0 if active else 6,
                2,
                0,
                0,
                0x0F,
                0,
            )
        )
        frame = decode_frame(raw)
        require_correlated(frame, OP_GET_ASSIST_CONFIG, 0x31)
        return frame

    @check("golden assist 0x01: requested SHADOW/effective DISABLED")
    def _assist_packed_01() -> None:
        status = parse_assist_status(assist_frame(0x01))
        require(status.requested_execution == ASSIST_EXEC_SHADOW, "low nibble")
        require(
            status.effective_execution == ASSIST_EXEC_DISABLED, "high nibble"
        )

    @check("golden assist 0x11: requested/effective SHADOW")
    def _assist_packed_11() -> None:
        status = parse_assist_status(assist_frame(0x11, active=True))
        require(status.requested_execution == ASSIST_EXEC_SHADOW, "low nibble")
        require(status.effective_execution == ASSIST_EXEC_SHADOW, "high nibble")

    @check("illegal assist 0x03 low nibble is rejected")
    def _assist_reject_03() -> None:
        _expect_parse_failure(
            lambda: parse_assist_status(assist_frame(0x03)),
            "reserved requested nibble 3 was accepted",
        )

    @check("illegal assist 0x30 high nibble is rejected")
    def _assist_reject_30() -> None:
        _expect_parse_failure(
            lambda: parse_assist_status(assist_frame(0x30)),
            "reserved effective nibble 3 was accepted",
        )

    @check("golden SET payloads: SHADOW and ACTIVE")
    def _golden_set_payloads() -> None:
        shadow = make_assist_set_payload(
            mode=1,
            execution=1,
            strategy=1,
            profile_id=0,
            left_level_percent=0,
            right_level_percent=0,
        )
        active = make_assist_set_payload(
            mode=1,
            execution=2,
            strategy=1,
            profile_id=0,
            left_level_percent=0,
            right_level_percent=0,
        )
        require(shadow == bytes.fromhex("01 01 01 01 00 00 00 00"), "SHADOW")
        require(active == bytes.fromhex("01 01 02 01 00 00 00 00"), "ACTIVE")

    for name, function in checks:
        try:
            function()
        except Exception as error:  # 单项失败后继续运行，方便一次看到所有回归结果。
            reporter.failed_test(name, str(error))
        else:
            reporter.passed_test(name)


async def run_hardware(args: argparse.Namespace, reporter: Reporter) -> None:
    """发现设备、订阅 Notify，然后运行只读套件和可选配置套件。"""

    # bleak 只在真机路径导入，所以离线 self-test 不要求安装蓝牙依赖。
    try:
        from bleak import BleakClient, BleakScanner
    except ImportError as error:
        raise AcceptanceError(
            "hardware run requires optional dependency 'bleak' "
            "(install with: python -m pip install bleak)"
        ) from error

    device = await resolve_device(
        BleakScanner, args.address, args.name, args.scan_timeout
    )
    reporter.passed_test(
        "BLE discovery",
        f"name={getattr(device, 'name', None)} "
        f"address={getattr(device, 'address', '?')}",
    )

    async with BleakClient(device, timeout=args.connect_timeout) as client:
        require(client.is_connected, "BLE connection was not established")
        reporter.passed_test("BLE connect")
        transport = BleTransport(client, args.response_timeout)
        await client.start_notify(
            RESPONSE_CHARACTERISTIC_UUID, transport.notification_callback
        )
        reporter.passed_test("response notifications enabled")
        ids = RequestIds()
        try:
            original = await run_read_only_tests(transport, reporter, ids)
            if args.verify_config:
                await run_config_verification(transport, reporter, ids, original)
        finally:
            await client.stop_notify(RESPONSE_CHARACTERISTIC_UUID)


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="ESP-EXO Stage 9 BLE acceptance (safe read-only by default)"
    )
    parser.add_argument(
        "--self-test",
        action="store_true",
        help="run offline golden-vector tests; does not import bleak",
    )
    target = parser.add_mutually_exclusive_group()
    target.add_argument("--address", help="BLE address (use when names collide)")
    target.add_argument(
        "--name",
        default=DEFAULT_DEVICE_NAME,
        help=f"exact BLE device name (default: {DEFAULT_DEVICE_NAME})",
    )
    parser.add_argument("--scan-timeout", type=float, default=10.0)
    parser.add_argument("--connect-timeout", type=float, default=10.0)
    parser.add_argument("--response-timeout", type=float, default=3.0)
    parser.add_argument(
        "--verify-config",
        action="store_true",
        help=(
            "while IDLE, temporarily verify TRANSPARENT+SHADOW and ACTIVE "
            "rejection, then restore the original selected config"
        ),
    )
    return parser


def validate_arguments(args: argparse.Namespace) -> None:
    for name in ("scan_timeout", "connect_timeout", "response_timeout"):
        require(
            getattr(args, name) > 0.0,
            f"--{name.replace('_', '-')} must be > 0",
        )
    require(
        not (args.self_test and args.verify_config),
        "--self-test and --verify-config cannot be used together",
    )


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_argument_parser()
    args = parser.parse_args(argv)
    reporter = Reporter()
    started = time.monotonic()

    try:
        validate_arguments(args)
    except AcceptanceError as error:
        reporter.failed_test("arguments", str(error))
        return reporter.summary()

    if args.self_test:
        run_self_test(reporter)
        print(f"elapsed={time.monotonic() - started:.3f}s")
        return reporter.summary()

    if args.verify_config:
        print(
            "[INFO] configuration verification enabled: IDLE is required; "
            "the original selected assist config will be restored in finally"
        )
    else:
        print(
            "[INFO] safe default: only GET_INFO, GET_GAIT_STATUS and "
            "GET_ASSIST_CONFIG are sent"
        )

    try:
        asyncio.run(run_hardware(args, reporter))
    except (AcceptanceError, OSError, asyncio.TimeoutError) as error:
        reporter.failed_test("hardware acceptance", str(error))
    except KeyboardInterrupt:
        reporter.failed_test("hardware acceptance", "interrupted by user")
    except Exception as error:  # bleak 后端在不同系统上会暴露不同异常类型。
        reporter.failed_test(
            "hardware acceptance", f"{type(error).__name__}: {error}"
        )

    print(f"elapsed={time.monotonic() - started:.3f}s")
    return reporter.summary()


if __name__ == "__main__":
    sys.exit(main())
