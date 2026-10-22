"""Bounded adaptive sphere unions for an explicitly estimated voxel belief.

No native scene shape enters this module. Every accepted refinement encloses
each assigned cell's bounding ball; this says nothing about unobserved matter.
Union volumes use common seeded samples, counting overlapping spheres once.
"""

from dataclasses import asdict, dataclass
import time

import numpy as np


@dataclass(frozen=True)
class FitLimits:
    voxel_m: float = .008
    padding_m: float = .003
    max_cells: int = 6000
    max_spheres: int = 10
    max_candidates: int = 3
    max_split_checks: int = 90
    volume_samples: int = 8192
    budget_s: float = .075
    sphere_penalty_m3: float = .000025
    minimum_gain_fraction: float = .015

    def __post_init__(self):
        positive = (self.voxel_m, self.budget_s, self.sphere_penalty_m3)
        if not np.isfinite(positive).all() or min(positive) <= 0 or not np.isfinite(self.padding_m) or self.padding_m < 0:
            raise ValueError('invalid metric/time fit limits')
        for name in ('max_cells', 'max_spheres', 'max_candidates', 'max_split_checks', 'volume_samples'):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 1:
                raise ValueError('fit work/count limits must be positive integers')
        if not 0 <= self.minimum_gain_fraction < 1:
            raise ValueError('invalid diminishing-gain threshold')


def box_belief(lower, upper, limits=FitLimits()):
    """Voxelize an estimated supported envelope; coarsen, never discard cells.

    The inferred envelope is a provisional completion hypothesis, not certified
    free/occupied space. Boundary cells may extend outside the input envelope.
    """
    lower, upper = np.asarray(lower, float), np.asarray(upper, float)
    if lower.shape != (3,) or upper.shape != (3,) or not np.isfinite([lower, upper]).all() or np.any(upper <= lower):
        raise ValueError('need finite positive estimated bounds')
    side = limits.voxel_m
    while True:
        counts = np.maximum(1, np.ceil((upper-lower)/side).astype(int))
        if np.prod(counts) <= limits.max_cells:
            break
        side *= 1.12
    axes = [lower[i]+(np.arange(n)+.5)*side for i, n in enumerate(counts)]
    cells = np.stack(np.meshgrid(*axes, indexing='ij'), -1).reshape(-1, 3)
    return cells, float(side)


def enclosing_ball(points, cell_radius):
    """Validated AABB-center enclosing ball; a heuristic, not minimum-ball code."""
    center = (points.min(0)+points.max(0))/2
    radius = np.nextafter(np.linalg.norm(points-center, axis=1).max()+cell_radius, np.inf)
    return center, float(radius)


def validate_cover(cells, side, centers, radii, padding=0.):
    cells, centers, radii = np.asarray(cells), np.asarray(centers), np.asarray(radii)
    if centers.shape != (len(radii), 3) or not len(radii) or not np.isfinite([*radii, *centers.ravel()]).all() or np.any(radii <= 0):
        raise ValueError('invalid sphere union')
    slack = radii[None]-np.linalg.norm(cells[:, None]-centers[None], axis=-1)-np.sqrt(3)*side/2-padding
    minimum = float(slack.max(1).min())
    if minimum < -1e-12:
        raise ValueError('sphere union does not enclose every whole belief cell')
    return minimum


def contains(points, centers, radii):
    return np.any(np.sum((points[:, None]-centers[None])**2, axis=-1) <= radii[None]**2, axis=1)


def adaptive_covers(cells, side, *, origin=(0., 0., 0.), limits=FitLimits()):
    start = time.perf_counter()
    cells = np.asarray(cells, float)
    if cells.ndim != 2 or cells.shape[1] != 3 or not 1 <= len(cells) <= limits.max_cells or not np.isfinite(cells).all() or not np.isfinite(side) or side <= 0:
        raise ValueError('invalid/oversized voxel belief')
    lattice = (cells-cells[0])/side
    if not np.allclose(lattice, np.round(lattice), atol=1e-8, rtol=0) or len(np.unique(cells, axis=0)) != len(cells):
        raise ValueError('belief cells must be unique cells on one non-overlapping cubic grid')
    ball_radius = np.sqrt(3)*side/2+limits.padding_m
    first = enclosing_ball(cells, ball_radius)
    # All later volumes use this fixed enclosing domain and identical samples.
    rng = np.random.default_rng(0)
    # A child's ball may protrude beyond its parent. Its radius is bounded by
    # the original AABB half-diagonal plus the cell ball radius, even for a
    # partial/non-box occupancy set. This contains ALL possible refinements.
    maximum_radius = np.linalg.norm((cells.max(0)-cells.min(0))/2)+ball_radius
    lower, upper = cells.min(0)-maximum_radius, cells.max(0)+maximum_radius
    samples = rng.uniform(lower, upper, (limits.volume_samples, 3))
    domain = float(np.prod(upper-lower))
    belief_volume = len(cells)*side**3  # Unique non-overlapping cells required.
    regions, balls = [np.arange(len(cells))], [first]
    inside = [contains(samples, np.array([first[0]]), np.array([first[1]]))]
    checks, history, stop = 0, [], 'sphere_cap'

    def record():
        centers, radii = np.array([b[0] for b in balls]), np.array([b[1] for b in balls])
        slack = validate_cover(cells, side, centers, radii, limits.padding_m)
        if np.any(centers-radii[:, None] < lower-1e-12) or np.any(centers+radii[:, None] > upper+1e-12):
            raise ValueError('union-volume domain does not enclose the completed spheres')
        occupied = np.logical_or.reduce(inside)
        fraction = occupied.mean()
        volume = float(domain*fraction)
        return dict(id=f'adaptive-{len(balls)}', centers_object_m=(centers-origin).tolist(),
                    radii_m=radii.tolist(), primitive_count=len(balls),
                    excess_volume_m3=max(0., volume-belief_volume),
                    excess_volume_standard_error_m3=float(domain*np.sqrt(fraction*(1-fraction)/len(samples))),
                    estimated_union_volume_m3=volume, minimum_cell_enclosure_slack_m=slack,
                    coverage='Every whole estimated belief cell plus explicit padding; unseen true-object coverage unproven.')

    history.append(record())
    while len(balls) < limits.max_spheres:
        if time.perf_counter()-start >= limits.budget_s:
            stop = 'cooperative_time_cap'; break
        current = np.logical_or.reduce(inside)
        best = None
        for i, indices in enumerate(regions):
            if len(indices) < 2:
                continue
            points = cells[indices]
            _, axes = np.linalg.eigh(np.cov(points.T)) if len(points) > 3 else (None, np.eye(3))
            for direction in axes.T:
                if checks >= limits.max_split_checks or time.perf_counter()-start >= limits.budget_s:
                    break
                checks += 1
                values = points@direction
                pivot = (values.min()+values.max())/2
                halves = [indices[values <= pivot], indices[values > pivot]]
                if any(not len(part) for part in halves):
                    continue
                children = [enclosing_ball(cells[part], ball_radius) for part in halves]
                masks = [contains(samples, np.array([b[0]]), np.array([b[1]])) for b in children]
                others = [mask for j, mask in enumerate(inside) if j != i]
                other = np.logical_or.reduce(others) if others else np.zeros(len(samples), bool)
                new = other | masks[0] | masks[1]
                gain = float((current.astype(int)-new).mean()*domain)
                if best is None or gain > best[0]:
                    best = (gain, i, halves, children, masks)
        if best is None:
            stop = 'work_cap' if checks >= limits.max_split_checks else 'unsplittable'; break
        gain, i, halves, children, masks = best
        if gain <= max(limits.sphere_penalty_m3, history[-1]['estimated_union_volume_m3']*limits.minimum_gain_fraction):
            stop = 'diminishing_union_gain'; break
        # Validate the completed replacement before publishing it.
        for part, ball in zip(halves, children, strict=True):
            validate_cover(cells[part], side, [ball[0]], [ball[1]], limits.padding_m)
        regions[i:i+1], balls[i:i+1], inside[i:i+1] = halves, children, masks
        history.append(record())
        if checks >= limits.max_split_checks:
            stop = 'work_cap'; break
    # Counts emerge from refinement. Bound the non-dominated proposal set while
    # preserving the baseline and the tightest completed result.
    chosen = np.unique(np.round(np.linspace(0, len(history)-1, min(limits.max_candidates, len(history)))).astype(int))
    candidates = [history[i] for i in chosen]
    elapsed = time.perf_counter()-start
    return candidates, dict(limits=asdict(limits), cells=len(cells), cell_side_m=side,
                           split_checks=checks, completed_counts=[c['primitive_count'] for c in history],
                           stop_reason=stop, elapsed_s=elapsed, budget_overrun=elapsed > limits.budget_s,
                           belief_volume_m3=belief_volume, unknown_region_policy='Estimated supported envelope hypothesis; unknown space is not certified free. No full true-object enclosure claim.')
