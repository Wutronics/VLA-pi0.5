"""Stable serial-port discovery for the Qnbot exoskeleton."""

from __future__ import annotations

from glob import glob
from pathlib import Path
from typing import Iterable


def select_serial_port(
    configured: str,
    *,
    stable_candidates: Iterable[str] | None = None,
    fallback_candidates: Iterable[str] | None = None,
) -> str:
    """Resolve ``auto`` to one unambiguous Qnbot/ACM serial device."""
    if configured != "auto":
        return configured

    stable = sorted(
        stable_candidates
        if stable_candidates is not None
        else glob("/dev/serial/by-id/*Qnbot*") + glob("/dev/serial/by-id/*qnbot*")
    )
    if len(stable) == 1:
        return stable[0]
    if len(stable) > 1:
        raise RuntimeError(f"发现多个 Qnbot 串口，请在配置中明确选择：{stable}")

    fallback = sorted(
        fallback_candidates
        if fallback_candidates is not None
        else glob("/dev/ttyACM*") + glob("/dev/ttyUSB*")
    )
    if len(fallback) == 1:
        return fallback[0]
    if not fallback:
        raise RuntimeError("未发现 Qnbot 串口（/dev/serial/by-id、ttyACM 或 ttyUSB）")
    raise RuntimeError(f"发现多个候选串口，请在配置中明确选择：{fallback}")


def check_serial_access(port: str) -> None:
    """Raise a useful error before pyserial attempts to open the device."""
    path = Path(port)
    if not path.exists():
        raise RuntimeError(f"串口不存在：{port}")
    if not path.is_char_device():
        # Stable /dev/serial/by-id symlinks resolve to character devices.
        resolved = path.resolve()
        if not resolved.is_char_device():
            raise RuntimeError(f"串口路径不是字符设备：{port}")
    import os

    if not os.access(port, os.R_OK | os.W_OK):
        raise PermissionError(
            f"当前用户无权读写 {port}；请加入 dialout 组后重新登录"
        )
