#!/usr/bin/env python3

import numpy as np


class Curve:
    TYPES = {
        "linear",
        "ease_in",
        "ease_out",
        "smooth_step",
        "smoother_step",
    }

    def __init__(self, curve_type="smooth_step"):
        if curve_type not in self.TYPES:
            raise ValueError(
                f"Unsupported curve type: {curve_type}"
            )

        self.curve_type = curve_type

    def evaluate(self, s):
        s = np.clip(s, 0.0, 1.0)

        if self.curve_type == "linear":
            return s

        if self.curve_type == "ease_in":
            return s**2

        if self.curve_type == "ease_out":
            return 1.0 - (1.0 - s)**2

        if self.curve_type == "smooth_step":
            return s**2 * (3.0 - 2.0 * s)

        if self.curve_type == "smoother_step":
            return s**3 * (
                s * (s * 6.0 - 15.0) + 10.0
            )

        return s