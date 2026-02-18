#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys


def ensure_task_prioritization_on_path(script_dir: str):
    """
    Adds TaskPrioritization parent folder to sys.path so `import TaskPrioritization...` works
    both in source tree and in installed layout inside the docker container.
    """
    here = os.path.abspath(script_dir)
    candidates = [
        "/TaskPrioritization",  # standard docker mount
        "/ros2_ws/TaskPrioritization",
        "/ros2_ws/src/TaskPrioritization",
        os.path.abspath(os.path.join(here, "..", "..", "..", "TaskPrioritization")),
        os.path.abspath(os.path.join(here, "..", "..", "TaskPrioritization")),
        os.path.abspath(os.path.join(here, "..", "TaskPrioritization")),
        os.path.abspath(os.path.join(here, "..", "..", "..", "..", "..", "TaskPrioritization")),
    ]

    for cand in candidates:
        if os.path.isdir(cand):
            parent = os.path.dirname(cand) or cand
            if parent not in sys.path:
                sys.path.append(parent)
            return cand

    return None
