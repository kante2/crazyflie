"""Connected body-sized trajectory geometry without ROS message dependencies.

The tube shares one ring at each curve joint and uses a transported frame.
Only its two ends receive hemisphere caps; intermediate samples never acquire
independent sphere or cylinder caps. Repeated stationary samples remain in the
time curve but are removed from the tube centerline.
"""

import math


def _point(value):
    if len(value) != 3:
        raise ValueError("a 3D center is required")
    result = tuple(float(component) for component in value)
    if not all(math.isfinite(component) for component in result):
        raise ValueError("centers must be finite")
    return result


def _dot(first, second):
    return sum(a * b for a, b in zip(first, second))


def _cross(first, second):
    return (
        first[1] * second[2] - first[2] * second[1],
        first[2] * second[0] - first[0] * second[2],
        first[0] * second[1] - first[1] * second[0],
    )


def _difference(first, second):
    return tuple(a - b for a, b in zip(first, second))


def _length(value):
    return math.sqrt(_dot(value, value))


def _unit(value):
    length = _length(value)
    if length < 1e-12:
        raise ValueError("a nonzero direction is required")
    return tuple(component / length for component in value)


def _project(reference, tangent):
    along = _dot(reference, tangent)
    return tuple(component - along * direction for component, direction in zip(reference, tangent))


def _frame(tangent, previous=None):
    if previous is None:
        # Choose the least parallel world axis, including vertical flights.
        axis = min(range(3), key=lambda index: abs(tangent[index]))
        reference = tuple(float(index == axis) for index in range(3))
        normal = _unit(_project(reference, tangent))
    else:
        normal = _project(previous[0], tangent)
        if _length(normal) < 1e-8:
            normal = _project(previous[1], tangent)
        normal = _unit(normal)
    binormal = _unit(_cross(tangent, normal))
    return normal, binormal


def _ring(center, normal, binormal, radius, sides):
    ring = []
    for index in range(sides):
        angle = 2 * math.pi * index / sides
        ring.append(tuple(c + radius * (math.cos(angle) * n + math.sin(angle) * b) for c, n, b in zip(center, normal, binormal)))
    return ring


def _connect(triangles, first, second):
    for index in range(len(first)):
        next_index = (index + 1) % len(first)
        triangles.extend((first[index], first[next_index], second[index]))
        triangles.extend((first[next_index], second[next_index], second[index]))


def _cap(triangles, equator, center, tangent, frame, radius, sides, steps, forward):
    previous = equator
    normal, binormal = frame
    sign = 1.0 if forward else -1.0
    for index in range(1, steps):
        angle = (math.pi / 2) * index / steps
        ring_center = tuple(c + sign * radius * math.sin(angle) * t for c, t in zip(center, tangent))
        current = _ring(ring_center, normal, binormal, radius * math.cos(angle), sides)
        if forward:
            _connect(triangles, previous, current)
        else:
            _connect(triangles, current, previous)
        previous = current
    tip = tuple(c + sign * radius * t for c, t in zip(center, tangent))
    for index in range(sides):
        if forward:
            triangles.extend((previous[index], previous[(index + 1) % sides], tip))
        else:
            triangles.extend((previous[index], tip, previous[(index + 1) % sides]))


def swept_tube(centers, radius, sides=12, cap_steps=3):
    """Return TRIANGLE_LIST xyz vertices of one capped, connected sweep.

    Every joint-ring vertex is exactly ``radius`` from its center. Each cap
    vertex is exactly ``radius`` from its endpoint; triangle interiors remain
    within the corresponding swept body envelope. A cusp or vertical segment
    changes the local frame rather than enlarging the radius with a miter.
    """
    radius = float(radius)
    if not math.isfinite(radius) or radius <= 0:
        raise ValueError("radius must be positive and finite")
    if type(sides) is not int or sides < 3 or type(cap_steps) is not int or cap_steps < 1:
        raise ValueError("sides >= 3 and cap_steps >= 1 are required")
    points = []
    for value in centers:
        point = _point(value)
        if not points or _length(_difference(point, points[-1])) > 1e-9:
            points.append(point)
    if len(points) < 2:
        return []
    directions = [_unit(_difference(end, start)) for start, end in zip(points, points[1:])]
    tangents = [directions[0]]
    for incoming, outgoing in zip(directions, directions[1:]):
        average = tuple(a + b for a, b in zip(incoming, outgoing))
        # At a reversal the bisector has no direction. Retain the incoming
        # frame at the shared joint instead of normalizing numerical noise.
        tangents.append(_unit(average) if _length(average) > 1e-6 else incoming)
    tangents.append(directions[-1])
    frames, rings = [], []
    for point, tangent in zip(points, tangents):
        frame = _frame(tangent, frames[-1] if frames else None)
        frames.append(frame)
        rings.append(_ring(point, frame[0], frame[1], radius, sides))
    triangles = []
    for first, second in zip(rings, rings[1:]):
        _connect(triangles, first, second)
    _cap(triangles, rings[0], points[0], tangents[0], frames[0], radius, sides, cap_steps, forward=False)
    _cap(triangles, rings[-1], points[-1], tangents[-1], frames[-1], radius, sides, cap_steps, forward=True)
    return triangles


def _times(trajectory):
    if not trajectory or not trajectory.get("times"):
        return []
    times = [float(t) for t in trajectory["times"]]
    if any(not math.isfinite(t) or t < 0 for t in times) or any(second <= first for first, second in zip(times, times[1:])):
        raise ValueError("trajectory times must be finite and increasing")
    return times


def curve_samples(trajectory, sample_dt=0.1):
    """Return all (relative seconds, exact quadratic position) curve samples.

    Merge a uniform display grid with the original optimization knots and final
    horizon. Coincident *times* are merged; coincident positions are retained so
    the full four-second time coloring survives a stopped forecast tail.
    """
    times = _times(trajectory)
    if not times:
        return []
    sample_dt = float(sample_dt)
    if not math.isfinite(sample_dt) or sample_dt <= 0:
        raise ValueError("sample_dt must be positive and finite")
    from .functions import sample_trajectory

    horizon = times[-1]
    original_times = set(times)
    candidates = sorted(times + [index * sample_dt for index in range(int(math.floor(horizon / sample_dt)) + 1)] + [horizon])
    merged = []
    for candidate in candidates:
        if merged and candidate - merged[-1] <= 1e-9:
            if candidate in original_times:
                merged[-1] = candidate
        else:
            merged.append(candidate)
    return [(elapsed, _point(sample_trajectory(trajectory, elapsed)["position"])) for elapsed in merged]


def time_annotations(trajectory, min_spacing=0.24):
    """Return '+1s' marks, grouping nearby consecutive marks into time ranges.

    A completely stationary curve has no extra time labels. A stationary tail
    keeps one '+2–4s' label rather than overlapping several texts at one point.
    A group's diameter stays small, so a slowly moving long curve cannot be
    collapsed merely because each consecutive pair is close.
    """
    times = _times(trajectory)
    if not times:
        return []
    min_spacing = float(min_spacing)
    if not math.isfinite(min_spacing) or min_spacing < 0:
        raise ValueError("min_spacing must be finite and nonnegative")
    samples = curve_samples(trajectory)
    origin = samples[0][1]
    if all(_length(_difference(position, origin)) <= 1e-6 for _, position in samples):
        return []
    from .functions import sample_trajectory

    horizon = times[-1]
    marks = [float(second) for second in range(1, int(math.floor(horizon)) + 1)]
    if horizon > 0 and (not marks or horizon - marks[-1] > 1e-9):
        marks.append(horizon)
    groups = []
    for elapsed in marks:
        position = _point(sample_trajectory(trajectory, elapsed)["position"])
        if groups and all(_length(_difference(position, previous_position)) <= min_spacing for _, previous_position in groups[-1]):
            groups[-1].append((elapsed, position))
        else:
            groups.append([(elapsed, position)])
    result = []
    for group in groups:
        first, last = group[0][0], group[-1][0]
        label = f"+{first:g}s" if len(group) == 1 else f"+{first:g}–{last:g}s"
        result.append((label, group[-1][1]))
    return result
