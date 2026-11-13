"""Offline, shape-preserving replay planning. Never opens robot hardware."""
from __future__ import annotations

import math
import numpy as np


CONTROL_HZ = 30.0
MAX_VELOCITY_RAD_S = .15
MAX_ACCELERATION_RAD_S2 = .30


class ReplayPath:
    """C1 monotone cubic path through every sample, slowed locally as needed.

    Start, end and photo markers have zero target velocity. No sample or photo
    index is removed. Each joint stays between its original segment endpoints.
    Limits apply to planned targets, not a claim about measured motor motion.
    """

    def __init__(self, samples, speed, stop_indices=()):
        self.q = np.asarray([s["q_rad"] for s in samples], dtype=float)
        self.times = np.asarray([s["t"] for s in samples], dtype=float)
        if (self.q.shape != (len(samples), 6) or len(samples) < 2
                or not np.all(np.isfinite(self.q)) or not np.all(np.isfinite(self.times))
                or np.any(np.diff(self.times) <= 0) or not math.isfinite(speed)
                or not .1 <= speed <= 1):
            raise ValueError("回放路径需要有效角度、递增时间及10%–100%速度")
        self.stops = {0, len(samples)-1, *stop_indices}
        self.durations = np.maximum(1., np.ceil(np.diff(self.times) / speed * CONTROL_HZ - 1e-10)) / CONTROL_HZ
        # Local stretching preserves recorded pauses without slowing the entire
        # action because of one fast hand-guided movement.
        for _ in range(12):
            self._coefficients()
            velocity, acceleration = self.peaks()
            factors = np.maximum(1., np.maximum(velocity / MAX_VELOCITY_RAD_S,
                                               np.sqrt(acceleration / MAX_ACCELERATION_RAD_S2)))
            if np.max(factors) <= 1.000001:
                break
            self.durations = np.ceil(self.durations * factors * CONTROL_HZ - 1e-10) / CONTROL_HZ
        # A uniform final stretch is an exact bound even if local stretching
        # changed derivatives at neighbouring segment boundaries.
        self._coefficients()
        velocity, acceleration = self.peaks()
        factor = max(1., float(np.max(velocity / MAX_VELOCITY_RAD_S)),
                     math.sqrt(float(np.max(acceleration / MAX_ACCELERATION_RAD_S2))))
        if factor > 1.000001:
            self.durations *= math.ceil(factor)
            self._coefficients()

    def _coefficients(self):
        h = self.durations[:, None]
        slopes = np.diff(self.q, axis=0) / h
        velocity = np.zeros_like(self.q)
        left, right = slopes[:-1], slopes[1:]
        same_direction = left * right > 0
        w1 = 2*h[1:] + h[:-1]
        w2 = h[1:] + 2*h[:-1]
        # Weighted harmonic mean: no overshoot, and zero derivative at reversals.
        denominator = w1 / np.where(same_direction, left, 1.) + w2 / np.where(same_direction, right, 1.)
        velocity[1:-1] = np.where(same_direction, (w1+w2) / np.where(same_direction, denominator, 1.), 0.)
        for index in self.stops:
            velocity[index] = 0.
        self.velocity = velocity
        self.a = 2*self.q[:-1] - 2*self.q[1:] + h*(velocity[:-1]+velocity[1:])
        self.b = -3*self.q[:-1] + 3*self.q[1:] - h*(2*velocity[:-1]+velocity[1:])
        self.c = h*velocity[:-1]

    def peaks(self):
        """Analytic per-segment maxima of |dq/dt| and |d²q/dt²|."""
        h = self.durations[:, None]
        vertex = np.clip(-self.b / np.where(np.abs(self.a) > 1e-20, 3*self.a, 1.), 0., 1.)
        v = np.maximum.reduce([np.abs(self.c/h), np.abs((3*self.a+2*self.b+self.c)/h),
                               np.abs((3*self.a*vertex**2+2*self.b*vertex+self.c)/h)])
        acc = np.maximum(np.abs(2*self.b/h**2), np.abs((6*self.a+2*self.b)/h**2))
        return np.max(v, axis=1), np.max(acc, axis=1)

    def position(self, segment, fraction):
        if fraction >= 1:
            return self.q[segment+1].copy()
        u = max(0., float(fraction))
        q = ((self.a[segment]*u + self.b[segment])*u + self.c[segment])*u + self.q[segment]
        # Round-off at a soft-limit endpoint must not create an overshoot.
        return np.clip(q, np.minimum(self.q[segment], self.q[segment+1]),
                       np.maximum(self.q[segment], self.q[segment+1]))

    @property
    def duration(self):
        return float(np.sum(self.durations))
