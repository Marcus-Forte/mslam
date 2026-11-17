"""Livox Mid360 reflectivity: what the `intensity` field of a point means, and which returns are detected.

Livox reports *calibrated reflectivity*, not raw return power, in 0..255:

* 0..150: diffuse targets, linear in reflectance (0 to 100 %). The range dependence of the return power is
  compensated by the sensor, so a surface reads the same at any distance. The incidence angle is not:
  a Lambertian surface returns power proportional to cos(incidence), so the reported value is
  `150 * reflectance * cos(incidence)`.
* 151..255: retro-reflectors (road signs, reflective tape). Modelled as `151 + 104 * cos(incidence)`.

Detection: the return power of a Lambertian target is proportional to `reflectance * cos / r^2`. The
datasheet gives the range at which a target is still detected: 40 m at 10 % and 70 m at 80 %. A pure
inverse-square law would give 113 m at 80 %, so the model fits a power law through the two datasheet
points instead: `r_max(s) = 40 m * (s / 0.1) ** k`, `k = ln(70/40) / ln(8) = 0.269`, with
`s = reflectance * cos(incidence)` (retro-reflectors count as `s = 1`). A hit beyond `r_max(s)` is a miss
(`(0,0,0)`, intensity 0), as in the real stream. Indoors this only removes returns at grazing incidence.

Not modelled: the weak returns below about 1 m in the real recordings (median intensity 4 to 5, probably
the operator), multiple returns, blooming around retro-reflectors, wet or specular surfaces.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .scene import Scene

DIFFUSE_FULL_SCALE = 150
RETRO_MIN, RETRO_MAX = 151, 255
RANGE_AT_10_PERCENT = 40.0   # m, datasheet
RANGE_AT_80_PERCENT = 70.0   # m, datasheet
RANGE_EXPONENT = math.log(RANGE_AT_80_PERCENT / RANGE_AT_10_PERCENT) / math.log(0.8 / 0.1)


@dataclass
class ReflectivityModel:
    reflectivity: np.ndarray        # (n_surfaces,) diffuse reflectance, 0..1, indexed like `Scene.boxes`
    retro: np.ndarray               # (n_surfaces,) bool
    noise: float = 0.0              # relative std of the reported reflectivity (speckle), 0 = off
    range_at_10: float = RANGE_AT_10_PERCENT

    @classmethod
    def from_scene(cls, scene: Scene, noise: float = 0.0) -> "ReflectivityModel":
        return cls(np.array([b.reflectivity for b in scene.boxes], float),
                   np.array([b.retro for b in scene.boxes], bool), noise)

    @classmethod
    def uniform(cls, n_surfaces: int, reflectivity: float = 0.4) -> "ReflectivityModel":
        return cls(np.full(n_surfaces, reflectivity), np.zeros(n_surfaces, bool))

    def detection_range(self, strength: np.ndarray) -> np.ndarray:
        """Largest range (m) at which a return of `strength` = reflectance * cos(incidence) is detected."""
        s = np.maximum(np.asarray(strength, float), 0.0)
        return self.range_at_10 * (s / 0.1) ** RANGE_EXPONENT

    def apply(self, r: np.ndarray, cos: np.ndarray, surface: np.ndarray, rng: np.random.Generator
              ) -> tuple[np.ndarray, np.ndarray]:
        """Detected mask and uint32 intensity for casts (`r` inf = no surface, `surface` = index or -1)."""
        hit = np.isfinite(r) & (surface >= 0)
        idx = np.where(hit, surface, 0)
        rho = self.reflectivity[idx]
        retro = self.retro[idx] & hit
        cos = np.clip(cos, 0.0, 1.0)
        strength = np.where(retro, 1.0, rho) * cos
        detected = hit & (np.where(hit, r, 0.0) <= self.detection_range(strength))
        gain = 1.0 + (rng.normal(0.0, self.noise, r.shape) if self.noise > 0.0 else 0.0)
        diffuse = np.clip(np.rint(DIFFUSE_FULL_SCALE * rho * cos * gain), 0, DIFFUSE_FULL_SCALE)
        retro_v = np.clip(np.rint(RETRO_MIN + (RETRO_MAX - RETRO_MIN) * cos * gain), RETRO_MIN, RETRO_MAX)
        intensity = np.where(detected, np.where(retro, retro_v, diffuse), 0).astype(np.uint32)
        return detected, intensity
