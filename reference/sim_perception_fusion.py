#!/usr/bin/env python3
"""Référence normative miroir de l'EKF 2D réduit ALG_PERCEPTION_FUSION."""

from __future__ import annotations

import math
import random
import sys

SCENARIOS = {"nominal", "outliers", "edge", "dropout"}


class AxisFilter:
    def __init__(self) -> None:
        self.pos = 0.0
        self.vel = 0.0
        self.p00 = 100.0
        self.p01 = 0.0
        self.p10 = 0.0
        self.p11 = 25.0

    def predict(self) -> None:
        self.pos += self.vel
        n00 = self.p00 + self.p01 + self.p10 + self.p11 + 0.04
        n01 = self.p01 + self.p11
        n10 = self.p10 + self.p11
        n11 = self.p11 + 0.01
        self.p00 = n00
        self.p01 = n01
        self.p10 = n10
        self.p11 = n11

    def update_position(self, measurement: float, variance: float, gated: bool) -> bool:
        residual = measurement - self.pos
        innovation_variance = self.p00 + variance
        if gated and abs(residual) > 4.0 * math.sqrt(innovation_variance):
            return False
        k0 = self.p00 / innovation_variance
        k1 = self.p10 / innovation_variance
        old_p00 = self.p00
        old_p01 = self.p01
        self.pos += k0 * residual
        self.vel += k1 * residual
        self.p00 = (1.0 - k0) * old_p00
        self.p01 = (1.0 - k0) * old_p01
        self.p10 -= k1 * old_p00
        self.p11 -= k1 * old_p01
        return True

    def update_velocity(self, measurement: float, variance: float) -> None:
        residual = measurement - self.vel
        innovation_variance = self.p11 + variance
        k0 = self.p01 / innovation_variance
        k1 = self.p11 / innovation_variance
        old_p10 = self.p10
        old_p11 = self.p11
        self.pos += k0 * residual
        self.vel += k1 * residual
        self.p00 -= k0 * old_p10
        self.p01 -= k0 * old_p11
        self.p10 = (1.0 - k1) * old_p10
        self.p11 = (1.0 - k1) * old_p11


def centered(rng: random.Random, amplitude: float) -> float:
    return (rng.random() * 2.0 - 1.0) * amplitude


def simulate(seed: int, scenario: str, steps: int) -> tuple[float, int, int, int, int]:
    rng = random.Random(seed)
    true_x = centered(rng, 12.0)
    true_y = centered(rng, 12.0)
    true_vx = centered(rng, 1.0)
    true_vy = centered(rng, 1.0)
    fx = AxisFilter()
    fy = AxisFilter()
    squared_error_sum = 0.0
    convergence_step = 0
    false_tracks = 0
    rejected_outliers = 0
    accepted_measurements = 0

    for step in range(1, steps + 1):
        true_vx += centered(rng, 0.08)
        true_vy += centered(rng, 0.08)
        true_x += true_vx
        true_y += true_vy
        fx.predict()
        fy.predict()

        odom_vx = true_vx + centered(rng, 0.30)
        odom_vy = true_vy + centered(rng, 0.30)
        fx.update_velocity(odom_vx, 0.09)
        fy.update_velocity(odom_vy, 0.09)
        accepted_measurements += 2

        gnss_available = scenario != "dropout" or step % 4 == 0
        if gnss_available:
            injected_outlier = scenario == "outliers" and step % 9 == 0
            offset = 30.0 if injected_outlier else 0.0
            gnss_x = true_x + centered(rng, 1.5) + offset
            gnss_y = true_y + centered(rng, 1.5) - offset
            accepted_x = fx.update_position(gnss_x, 2.25, True)
            accepted_y = fy.update_position(gnss_y, 2.25, True)
            accepted_measurements += int(accepted_x) + int(accepted_y)
            rejected_outliers += int(not accepted_x) + int(not accepted_y)
            if injected_outlier and (accepted_x or accepted_y):
                false_tracks += 1

        if scenario == "edge" and step % 4 == 0:
            edge_x = true_x + centered(rng, 0.6)
            edge_y = true_y + centered(rng, 0.6)
            accepted_measurements += int(fx.update_position(edge_x, 0.36, True))
            accepted_measurements += int(fy.update_position(edge_y, 0.36, True))

        dx = fx.pos - true_x
        dy = fy.pos - true_y
        squared_error = dx * dx + dy * dy
        squared_error_sum += squared_error
        if convergence_step == 0 and math.sqrt(squared_error) <= 2.0:
            convergence_step = step

    rmse = math.sqrt(squared_error_sum / float(steps))
    return rmse, convergence_step, false_tracks, rejected_outliers, accepted_measurements


def main(argv: list[str]) -> int:
    if len(argv) != 4:
        print(f"Usage: {argv[0]} <seed> <scenario: nominal|outliers|edge|dropout> <steps>", file=sys.stderr)
        return 2
    try:
        seed = int(argv[1])
        scenario = argv[2]
        steps = int(argv[3])
    except ValueError as exc:
        print(f"argument invalide: {exc}", file=sys.stderr)
        return 2
    if seed < 0 or seed > (1 << 64) - 1 or scenario not in SCENARIOS or steps <= 0:
        print("arguments hors domaine", file=sys.stderr)
        return 2

    rmse, convergence, false_tracks, rejected, accepted = simulate(seed, scenario, steps)
    print("seed,scenario,steps,rmse,convergence_step,false_tracks,rejected_outliers,accepted_measurements")
    print(f"{seed},{scenario},{steps},{rmse:.12f},{convergence},{false_tracks},{rejected},{accepted}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
