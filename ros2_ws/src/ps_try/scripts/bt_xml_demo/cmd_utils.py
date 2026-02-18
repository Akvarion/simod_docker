#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

from typing import List

import numpy as np


def cmd_slice(cmd, start: int, end: int, expected_len: int) -> List[float]:
    if cmd is None:
        return [0.0] * expected_len
    try:
        out = [float(v) for v in cmd[start:end]]
    except Exception:
        out = []
    if len(out) < expected_len:
        out.extend([0.0] * (expected_len - len(out)))
    return out[:expected_len]


def sanitize_arm_cmd(values: List[float], abs_max: float) -> List[float]:
    out = []
    lim = max(float(abs_max), 1e-3)
    for v in values:
        vv = float(v) if np.isfinite(v) else 0.0
        if vv > lim:
            vv = lim
        elif vv < -lim:
            vv = -lim
        out.append(vv)
    return out


def sanitize_base_cmd(values: List[float], xy_abs_max: float, wz_abs_max: float, ramp: float = 1.0) -> List[float]:
    if len(values) < 3:
        values = list(values) + [0.0] * (3 - len(values))
    r = min(max(float(ramp), 0.0), 1.0)
    lim_xy = max(float(xy_abs_max), 1e-3) * r
    lim_wz = max(float(wz_abs_max), 1e-3) * r

    out = []
    for i, v in enumerate(values[:3]):
        vv = float(v) if np.isfinite(v) else 0.0
        lim = lim_xy if i < 2 else lim_wz
        if vv > lim:
            vv = lim
        elif vv < -lim:
            vv = -lim
        out.append(vv)
    return out
