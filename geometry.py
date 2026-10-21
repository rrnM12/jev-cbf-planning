"""Object-local sphere covers and an isolated ground-truth geometry experiment.

All lengths are metres. The bounding box is centred at the object-frame origin.
No robot size, clearance margin, perception uncertainty, or CBF is included here.
"""

from dataclasses import dataclass, asdict
from numbers import Integral

import numpy as np


@dataclass(frozen=True)
class SphereCover:
    candidate_id: str
    centers_m: tuple[tuple[float, float, float], ...]
    radii_m: tuple[float, ...]
    split_axis: int


def _half_extents(values):
    half_extents = np.asarray(values, dtype=float)
    if (
        half_extents.shape != (3,)
        or not np.isfinite(half_extents).all()
        or np.any(half_extents <= 0)
    ):
        raise ValueError("half_extents_m must contain three finite, positive lengths")
    return half_extents


def _positive_integer(value, name):
    if isinstance(value, bool) or not isinstance(value, Integral) or value <= 0:
        raise ValueError(f"{name} must be a positive integer")


def make_sphere_covers(half_extents_m, counts=(1, 2, 4)) -> tuple[SphereCover, ...]:
    """Enclose each of N equal box sections with its circumscribing sphere.

    Split along the longest axis (first axis wins ties). Every section is inside
    its sphere, so the UNION covers the entire box, including its interior, and
    thus any object contained in that box. This is a construction-based argument,
    not a conclusion drawn from sampled points. Zero/negative extents are rejected
    rather than silently inflated. Radii are rounded outward by one float64 ULP;
    this is numerical padding, not a physical clearance margin or a formal
    floating-point certificate.
    """
    half_extents = _half_extents(half_extents_m)
    counts = tuple(counts)
    if not counts:
        raise ValueError("counts must not be empty")
    for count in counts:
        _positive_integer(count, "sphere count")
    if len(set(counts)) != len(counts):
        raise ValueError("sphere counts must be unique")

    axis = int(np.argmax(half_extents))
    candidates = []
    for count in counts:
        edges = np.linspace(-half_extents[axis], half_extents[axis], count + 1)
        centers = np.zeros((count, 3))
        centers[:, axis] = edges[:-1] / 2 + edges[1:] / 2
        sections = np.tile(half_extents, (count, 1))
        sections[:, axis] = np.maximum(
            centers[:, axis] - edges[:-1], edges[1:] - centers[:, axis]
        )
        radii = np.nextafter(np.hypot.reduce(sections, axis=1), np.inf)
        if not np.isfinite(radii).all() or np.any(sections <= 0):
            raise ValueError("dimensions/count produce unrepresentable sphere sections")
        candidates.append(SphereCover(
            candidate_id=f"box-split-{count}",
            centers_m=tuple(tuple(float(x) for x in center) for center in centers),
            radii_m=tuple(float(radius) for radius in radii),
            split_axis=axis,
        ))
    return tuple(candidates)


def make_shape_covers(half_extents_m, *, shape, counts=(1, 2, 4)):
    """Add shape-aware alternatives without removing the original box covers.

    Cylinder slabs use the exact circular cross-section, rather than its square
    bound. A four-cell box grid splits the two longest axes (split_axis=-1).
    Each cell/slab is enclosed, so the union contains the entire stated shape.
    These are deterministic candidates, not model choices or perception fitting.
    """
    half = _half_extents(half_extents_m)
    counts = tuple(counts)
    covers = list(make_sphere_covers(half, counts))
    if shape == "cylinder":
        if half[0] != half[1]:
            raise ValueError("circular cylinder requires equal X/Y dimensions")
        for count in counts:
            edges = np.linspace(-half[2], half[2], count+1)
            z = edges[:-1]/2+edges[1:]/2
            dz = np.maximum(z-edges[:-1], edges[1:]-z)
            radii = np.nextafter(np.hypot(half[0], dz), np.inf)
            covers.append(SphereCover(f"cylinder-split-{count}", tuple((0., 0., float(v)) for v in z), tuple(map(float, radii)), 2))
    elif shape == "box":
        if 4 in counts:
            from itertools import product
            axes = np.argsort(-half, kind="stable")[:2]
            cell = half.copy(); cell[axes] /= 2
            radius = float(np.nextafter(np.hypot.reduce(cell), np.inf))
            points = []
            for signs in product((-1, 1), repeat=2):
                center = np.zeros(3); center[axes] = np.array(signs)*half[axes]/2
                points.append(tuple(map(float, center)))
            covers.append(SphereCover("box-grid-4", tuple(points), (radius,)*4, -1))
    else:
        raise ValueError("shape must be box or cylinder")
    return tuple(covers)


def sphere_union_contains(points_m, cover: SphereCover) -> np.ndarray:
    """Test points against the sphere union, counting overlaps only once."""
    points = np.asarray(points_m, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError("points_m must have shape (N, 3) with finite coordinates")
    inside = np.zeros(len(points), dtype=bool)
    for center, radius in zip(cover.centers_m, cover.radii_m, strict=True):
        inside |= np.hypot.reduce((points - center) / radius, axis=1) <= 1
    return inside


def evaluate_covers(
    half_extents_m, covers, *, shape="box", samples=100_000, seed=0
) -> dict:
    """Estimate union/excess volumes using shared seeded Monte Carlo points.

    Reference geometry is an exact box or a Z-aligned circular cylinder whose
    bounding box has the supplied half-extents. It is independent of the sphere
    membership test. Sampled misses are a diagnostic, NOT a coverage proof.
    Standard errors quantify sampling noise only, not model/geometry uncertainty.
    """
    half_extents = _half_extents(half_extents_m)
    _positive_integer(samples, "samples")
    covers = tuple(covers)
    if not covers:
        raise ValueError("at least one cover is required")
    if shape not in ("box", "cylinder"):
        raise ValueError("shape must be 'box' or 'cylinder'")
    if shape == "cylinder" and half_extents[0] != half_extents[1]:
        raise ValueError("cylinder X and Y dimensions must be equal")

    # One common domain enclosing ALL candidates; chains can extend farther along
    # the long axis than the single sphere. Do not use just the object's box.
    centers = np.concatenate([c.centers_m for c in covers])
    radii = np.concatenate([c.radii_m for c in covers])[:, None]
    lower = np.min(centers - radii, axis=0)
    upper = np.max(centers + radii, axis=0)
    domain_volume = float(np.prod(upper - lower))
    if not np.isfinite(domain_volume) or domain_volume <= 0:
        raise ValueError("sampling domain volume must be finite and positive")
    points = np.random.default_rng(seed).uniform(lower, upper, size=(samples, 3))
    if shape == "box":
        object_mask = np.all(np.abs(points) <= half_extents, axis=1)
        object_volume = float(8 * np.prod(half_extents))
    else:
        object_mask = (
            np.hypot(points[:, 0], points[:, 1]) <= half_extents[0]
        ) & (np.abs(points[:, 2]) <= half_extents[2])
        object_volume = float(np.pi * half_extents[0] ** 2 * 2 * half_extents[2])

    def estimate(mask):
        fraction = float(np.mean(mask))
        return (
            domain_volume * fraction,
            domain_volume * np.sqrt(fraction * (1 - fraction) / samples),
        )

    results = []
    for cover in covers:
        covered = sphere_union_contains(points, cover)
        volume, volume_se = estimate(covered)
        excess, excess_se = estimate(covered & ~object_mask)
        results.append(asdict(cover) | {
            "primitive_count": len(cover.radii_m),
            "union_volume_m3": volume,
            "union_volume_standard_error_m3": float(volume_se),
            "excess_volume_m3": excess,
            "excess_volume_standard_error_m3": float(excess_se),
            "uncovered_object_samples": int(np.count_nonzero(object_mask & ~covered)),
        })
    return {
        "frame": "object-local, box centred at origin; cylinder axis is Z",
        "half_extents_m": half_extents.tolist(),
        "reference_shape": shape,
        "object_volume_m3": object_volume,
        "samples": int(samples),
        "seed": seed,
        "sampling_aabb_m": [lower.tolist(), upper.tolist()],
        "object_samples": int(np.count_nonzero(object_mask)),
        "candidates": results,
    }
