"""Central-PC implementation of per-agent distributed model predictive control.

Each robot solves its own acceleration QP against communicated neighbour
forecasts. Only the first detected collision with each neighbour creates a
separating half-space; forecasts are exchanged and refined before the batch is
released. A continuous swept-volume check gates the *whole* common-time batch.
Unlike the paper, this adaptation uses hard collision constraints and a measured
state double integrator instead of soft Bezier reference constraints and an
identified Crazyflie tracking model. It is not a claim of paper reproduction or
of completeness in arbitrary scenarios. Static and prescribed moving boxes use
conservative enclosing ellipsoids and continuous-time relative-motion checks.
"""

import math
import time as wall_time
from itertools import combinations

import numpy as np

from .qp import AgentQP
from ..common.obstacles import geometry, validate_position, motion_state, motion_breaks

DEFAULT_CONFIG = {
    'frame': 'world', 'dt': .2, 'horizon': 4.0, 'speed': 2.0,
    'acceleration': .6, 'deceleration': .8,
    'bounds': [[-4.5, -4.5, .2], [4.5, 4.5, 3.5]],
    'horizontal_separation': .52, 'vertical_separation': 1.1,
    'collision_margin': .06, 'state_timeout': .5, 'command_ttl': .4,
    'exchange_iterations': 8, 'tracking_reset_distance': .15,
    'tracking_reset_velocity': .3,
}


def sample_trajectory(trajectory, elapsed):
    """Evaluate exact constant-acceleration position/velocity, holding its end."""
    times = trajectory['times']
    if elapsed < 0:
        elapsed = 0.0
    k = max(0, min(len(times)-1, int(np.searchsorted(times, elapsed, side='right'))-1))
    p = np.asarray(trajectory['positions'][k], dtype=float)
    v = np.asarray(trajectory['velocities'][k], dtype=float)
    a = np.asarray(trajectory['accelerations'][k], dtype=float)
    t = min(max(0.0, elapsed-times[k]), times[k+1]-times[k]) if k+1 < len(times) else 0.0
    return {'position': (p+t*v+.5*t*t*a).tolist(),
            'velocity': (v+t*a).tolist(), 'acceleration': a.tolist()}


def _at(trajectory, time, dt):
    k = min(len(trajectory['positions'])-2, max(0, int(time/dt)))
    t = min(dt, max(0.0, time-k*dt))
    return trajectory['positions'][k] + trajectory['velocities'][k]*t + .5*trajectory['accelerations'][k]*t*t


def _segment_minimum(r, v, a, duration):
    """Exact minimum of ||r+v*t+a*t²/2|| using the cubic derivative."""
    # Descending powers of d/dt ||r+v*t+a*t²/2||².
    polynomial = np.asarray([a@a, 3*v@a, 2*(v@v+r@a), 2*r@v])
    nonzero = np.flatnonzero(np.abs(polynomial) > 1e-12)
    roots = np.roots(polynomial[nonzero[0]:]) if len(nonzero) else []
    candidates = [0.0, duration]
    candidates.extend(float(root.real) for root in roots
                      if abs(root.imag) < 1e-8 and 0 < root.real < duration)
    distances = [np.linalg.norm(r+v*t+.5*a*t*t) for t in candidates]
    index = int(np.argmin(distances))
    return float(distances[index]), candidates[index]


def swept_pair(first, second, dt, scale, threshold=1.0):
    """Return min scaled distance and first unsafe segment's closest time."""
    minimum, collision_time = math.inf, None
    for k in range(len(first['positions'])-1):
        r = (first['positions'][k]-second['positions'][k])*scale
        v = (first['velocities'][k]-second['velocities'][k])*scale
        a = (first['accelerations'][k]-second['accelerations'][k])*scale
        distance, within = _segment_minimum(r, v, a, dt)
        minimum = min(minimum, distance)
        if collision_time is None and distance < threshold:
            collision_time = k*dt+within
    return minimum, collision_time


class MultiDroneTrajectoryPlanner:
    """Receding-horizon per-robot optimization with atomic safety validation."""

    def __init__(self, config=None):
        self.config = dict(DEFAULT_CONFIG, **(config or {}))
        self.dt = float(self.config['dt'])
        self.steps = int(round(self.config['horizon']/self.dt))
        for name in ('dt', 'horizon', 'speed', 'acceleration', 'deceleration',
                     'horizontal_separation', 'vertical_separation', 'state_timeout',
                     'command_ttl', 'tracking_reset_distance', 'tracking_reset_velocity'):
            if not math.isfinite(self.config[name]) or self.config[name] <= 0:
                raise ValueError(f'{name} must be a finite positive value')
        if self.steps < 2 or abs(self.steps*self.dt-self.config['horizon']) > 1e-8:
            raise ValueError('horizon must be an integer multiple of dt, with at least two steps')
        if type(self.config['exchange_iterations']) is not int or self.config['exchange_iterations'] < 1:
            raise ValueError('exchange_iterations must be a positive integer')
        if not math.isfinite(self.config['collision_margin']) or self.config['collision_margin'] < 0:
            raise ValueError('collision_margin cannot be negative')
        bounds = np.asarray(self.config['bounds'], dtype=float)
        if bounds.shape != (2,3) or not np.all(np.isfinite(bounds)) or np.any(bounds[0] >= bounds[1]):
            raise ValueError('bounds must contain ordered finite 3D limits')
        self.qp = AgentQP(self.steps, self.dt, self.config)
        self.sequence, self.previous = 0, None
        self.obstacle_time = 0.0
        self.goals = {}
        self.last_accelerations = {}

    def _base(self, now, status, trajectories, diagnostics):
        self.sequence += 1
        return {'version': 1, 'algorithm': 'DMPC_ON_DEMAND', 'frame': self.config['frame'],
                'sequence': self.sequence, 'stamp': now, 'issued_at': now,
                'valid_until': now+self.config['command_ttl'], 'dt': self.dt,
                'horizon': self.steps*self.dt, 'status': status,
                'trajectories': trajectories, 'diagnostics': diagnostics}

    def waiting(self, now, reason='waiting for fresh airborne states and complete goals'):
        """Public synchronized no-command envelope for ROS readiness gates."""
        return self._base(now, 'WAITING', {}, {'solve_ms': 0.0, 'reason': reason})

    def _scale(self, first, second, states):
        horizontal = max(self.config['horizontal_separation'],
                         states[first].get('radius', 0.0)+states[second].get('radius', 0.0))
        uncertainty = (states[first].get('_clock_uncertainty', 0.0)+states[second].get('_clock_uncertainty', 0.0)
                       + states[first].get('_tracking_uncertainty', 0.0)+states[second].get('_tracking_uncertainty', 0.0))
        # An acceleration-bounded ball of clock-extrapolation error inflates
        # the ellipsoid by a common factor; its smallest (horizontal) axis
        # then grows by at least the sum of both agents' uncertainty radii.
        factor = 1+uncertainty/horizontal
        return np.asarray([1/(horizontal*factor), 1/(horizontal*factor),
                           1/(self.config['vertical_separation']*factor)])

    def _validate(self, forecasts, states, threshold=1.0):
        minimum, collisions = math.inf, []
        for first, second in combinations(sorted(forecasts), 2):
            scale = self._scale(first, second, states)
            distance, collision_time = swept_pair(forecasts[first], forecasts[second], self.dt, scale, threshold)
            minimum = min(minimum, distance)
            if collision_time is not None:
                collisions.append((first, second, collision_time))
        return minimum, collisions

    def _obstacle_geometry(self, obstacle, state, when=0.):
        uncertainty = state.get('_clock_uncertainty', 0.) + state.get('_tracking_uncertainty', 0.)
        return geometry(obstacle, max(state.get('radius', 0.), self.config.get('robot_radius', .18)),
                        self.config['collision_margin'], uncertainty, stamp=self.obstacle_time+when)

    def _obstacle_collisions(self, forecasts, states):
        minimum, collisions = math.inf, []
        for name, forecast in forecasts.items():
            for obstacle in self.config.get('obstacles', []):
                _, scale = self._obstacle_geometry(obstacle, states[name])
                for k in range(self.steps):
                    begin = self.obstacle_time+k*self.dt
                    cuts = motion_breaks(obstacle, begin, begin+self.dt)
                    for left, right in zip(cuts, cuts[1:]):
                        offset = left-begin
                        p = forecast['positions'][k]+offset*forecast['velocities'][k]+.5*offset**2*forecast['accelerations'][k]
                        v = forecast['velocities'][k]+offset*forecast['accelerations'][k]
                        center, _ = motion_state(obstacle, left)
                        # Midpoint velocity avoids floating-point ambiguity at a reversal.
                        _, obstacle_v = motion_state(obstacle, (left+right)/2)
                        distance, within = _segment_minimum((p-center)*scale,
                            (v-obstacle_v)*scale, forecast['accelerations'][k]*scale, right-left)
                        minimum = min(minimum, distance)
                        if distance < 1-2e-4:
                            collisions.append((name, obstacle, k*self.dt+offset+within))
        # All offending segments are retained, including direction reversals.
        return minimum, collisions

    def _brake(self, states):
        forecasts = {}
        for name, state in states.items():
            p, v = np.asarray(state['position'], dtype=float), np.asarray(state['velocity'], dtype=float)
            positions, velocities, accelerations = [p.copy()], [v.copy()], []
            for _ in range(self.steps):
                speed = np.linalg.norm(v)
                a = -v * min(1/self.dt, self.config['deceleration']/speed) if speed > 1e-10 else np.zeros(3)
                p = p+v*self.dt+.5*a*self.dt**2
                v = v+a*self.dt
                positions.append(p.copy()); velocities.append(v.copy()); accelerations.append(a.copy())
            forecasts[name] = {'positions': np.asarray(positions), 'velocities': np.asarray(velocities),
                               'accelerations': np.vstack((accelerations, np.zeros(3)))}
        return forecasts

    def _wire(self, forecasts, goals):
        result = {}
        for name, forecast in forecasts.items():
            result[name] = {key: value.tolist() for key, value in forecast.items()}
            result[name]['times'] = [round(k*self.dt, 10) for k in range(self.steps+1)]
            goal = goals.get(name)
            if isinstance(goal, dict):
                result[name]['goal_id'] = goal.get('goal_id', '')
                result[name]['goal'] = list(goal['position'])
            elif goal is not None:
                result[name]['goal'] = list(goal)
            result[name]['yaw'] = 0.0
        return result

    def _bounded(self, forecasts, braking=False):
        # Workspace bounds must hold *between* knots too, where a reversing
        # component can reach an extremum beyond both of its endpoint positions.
        low, high = np.asarray(self.config['bounds'])
        for forecast in forecasts.values():
            for key in ('positions', 'velocities', 'accelerations'):
                if not np.all(np.isfinite(forecast[key])):
                    return False
            acceleration_limit = self.config['deceleration'] if braking else self.config['acceleration']
            speed_limit = max(self.config['speed'], np.linalg.norm(forecast['velocities'][0])) if braking else self.config['speed']
            if (np.max(np.linalg.norm(forecast['accelerations'], axis=1)) > acceleration_limit+2e-4
                or np.max(np.linalg.norm(forecast['velocities'], axis=1)) > speed_limit+2e-4
                or np.linalg.norm(forecast['velocities'][-1]) > 2e-4):
                return False
            if np.any(forecast['positions'] < low-1e-4) or np.any(forecast['positions'] > high+1e-4):
                return False
            for p, v, a in zip(forecast['positions'][:-1], forecast['velocities'][:-1], forecast['accelerations'][:-1]):
                for axis in range(3):
                    if abs(a[axis]) < 1e-12:
                        continue
                    t = -v[axis]/a[axis]
                    if 0 < t < self.dt:
                        value = p[axis]+v[axis]*t+.5*a[axis]*t*t
                        if value < low[axis]-1e-4 or value > high[axis]+1e-4:
                            return False
        return True

    def plan(self, states, goals, now):
        started = wall_time.perf_counter()
        diagnostics = {'iterations': 0, 'qp_status': {}, 'event_resets': [], 'collision_constraints': 0, 'symmetry_guides': [], 'initialization': {}}
        if not math.isfinite(now):
            raise ValueError('now must be finite')
        if not states or set(states) != set(goals):
            return self._base(now, 'STALE', {}, dict(diagnostics, reason='missing state or goal', solve_ms=0.0))
        self.obstacle_time = now
        positions, velocities, targets = {}, {}, {}
        for name, state in states.items():
            try:
                p = np.asarray(state['position'], dtype=float)
                v = np.asarray(state['velocity'], dtype=float)
                stamp = float(state['stamp'])
                goal = goals[name]['position'] if isinstance(goals[name], dict) else goals[name]
                target = np.asarray(goal, dtype=float)
                if any(x.shape != (3,) or not np.all(np.isfinite(x)) for x in (p, v, target)):
                    raise ValueError('invalid vectors')
                if not math.isfinite(stamp) or now-stamp > self.config['state_timeout'] or stamp-now > .05:
                    raise ValueError('stale state')
                if state.get('frame', self.config['frame']) != self.config['frame']:
                    raise ValueError('frame mismatch')
                low, high = np.asarray(self.config['bounds'])
                if np.any(target < low) or np.any(target > high):
                    raise ValueError('goal outside workspace')
                validate_position(target, self.config)
                radius = state.get('radius', 0.0)
                if not math.isfinite(radius) or radius < 0:
                    raise ValueError('invalid radius')
                # Put all measured states on the batch clock before predicting.
                # Constant-velocity propagation covers short measurement latency;
                # collision_margin absorbs the model/tracking uncertainty.
                positions[name], velocities[name], targets[name] = p+v*max(0.0, now-stamp), v, target
            except (KeyError, ValueError, TypeError):
                diagnostics['reason'] = f'invalid or stale state/goal for {name}'
                diagnostics['solve_ms'] = (wall_time.perf_counter()-started)*1000
                return self._base(now, 'STALE', {}, diagnostics)
        states = {name: dict(state, _clock_uncertainty=.5*max(self.config['acceleration'], self.config['deceleration'])*max(0.0, now-float(state['stamp']))**2)
                  for name, state in states.items()}
        measured_positions = {name: p.copy() for name, p in positions.items()}
        measured_velocities = {name: v.copy() for name, v in velocities.items()}
        forecasts, constraints = {}, {name: [] for name in states}
        for name in sorted(states):
            reference = None
            elapsed = now-self.previous['stamp'] if self.previous else math.inf
            if self.previous and name in self.previous['trajectories']:
                reference = sample_trajectory(self.previous['trajectories'][name], elapsed)
                if (np.linalg.norm(positions[name]-reference['position']) > self.config['tracking_reset_distance']
                    or np.linalg.norm(velocities[name]-reference['velocity']) > self.config['tracking_reset_velocity']
                    or not 0 <= elapsed <= self.config['command_ttl']):
                    diagnostics['event_resets'].append(name)
            if name in self.goals and not np.allclose(self.goals[name], targets[name], atol=1e-8):
                diagnostics['event_resets'].append(name)
            reset = name in diagnostics['event_resets']
            if reference is not None and not reset:
                # Keep the accepted input continuous during normal tracking.
                # Reanchoring it to a biased position measurement at every tick
                # can ratchet the reference down despite an unchanged altitude
                # goal. This follows the paper's nominal/event-reset distinction.
                positions[name] = np.asarray(reference['position'], dtype=float)
                velocities[name] = np.asarray(reference['velocity'], dtype=float)
                states[name]['_tracking_uncertainty'] = float(np.linalg.norm(measured_positions[name]-positions[name]))
                diagnostics['initialization'][name] = 'nominal'
            else:
                diagnostics['initialization'][name] = 'measured'
            reset = name in diagnostics['event_resets']
            initial = None if reset else self.last_accelerations.get(name)
            forecast, status = self.qp.solve(positions[name], velocities[name], targets[name], initial=initial)
            diagnostics['qp_status'][name] = status
            if forecast is None:
                break
            forecasts[name] = forecast
        threshold = 1 + self.config['collision_margin']/self.config['horizontal_separation']
        qp_targets = dict(targets)
        if len(forecasts) == len(states):
            _, nominal_collisions = self._validate(forecasts, states, threshold)
            affected = {name for first, second, _ in nominal_collisions for name in (first, second)}
            _, obstacle_collisions = self._obstacle_collisions(forecasts, states)
            obstacle_affected = {name for name, _, _ in obstacle_collisions}
            affected.update(obstacle_affected)
            # A vanishing planar guide breaks symmetric crossings without giving
            # any agent right-of-way or changing its final altitude/goal.
            diagnostics['symmetry_guides'] = sorted(affected)
            for name in sorted(affected):
                direction = targets[name]-positions[name]
                tangent = np.asarray([-direction[1], direction[0], 0.0])
                tangent /= max(np.linalg.norm(tangent), 1e-12)
                profile = np.sin(np.pi*np.arange(1, self.steps+1)/self.steps)
                qp_targets[name] = forecasts[name]['positions'][1:]+(1.8 if name in obstacle_affected else 1.2)*profile[:, None]*tangent
                forecast, qp_status = self.qp.solve(positions[name], velocities[name], qp_targets[name], initial=self.last_accelerations.get(name))
                diagnostics['qp_status'][name] = qp_status
                if forecast is not None:
                    forecasts[name] = forecast
        status = 'SOLVED'
        if len(forecasts) == len(states):
            for iteration in range(self.config['exchange_iterations']+1):
                minimum, collisions = self._validate(forecasts, states, threshold-2e-4)
                diagnostics['iterations'] = iteration
                _, obstacle_collisions = self._obstacle_collisions(forecasts, states)
                if not collisions and not obstacle_collisions and self._bounded(forecasts):
                    break
                if iteration == self.config['exchange_iterations']:
                    status = 'BRAKING'; break
                # Gauss-Seidel exchange: each QP reads the most recent communicated
                # predictions. Add the earliest offending sample of each pair only.
                failed = False
                for name in sorted(states):
                    neighbours = []
                    for other in sorted(states):
                        if other == name:
                            continue
                        scale = self._scale(name, other, states)
                        _, when = swept_pair(forecasts[name], forecasts[other], self.dt, scale, threshold)
                        if when is None or when <= 1e-6:
                            continue
                        own, neighbour = _at(forecasts[name], when, self.dt), _at(forecasts[other], when, self.dt)
                        relative = (own-neighbour)*scale
                        if np.linalg.norm(relative) < .05:
                            relative = (positions[name]-positions[other])*scale
                        # A common planar handedness resolves perfectly symmetric
                        # head-on exchanges, while the same-altitude problem stays
                        # planar instead of escaping above another drone.
                        normal = relative.copy()
                        if iteration == 0 and np.linalg.norm(normal[:2]) > 1e-8:
                            normal[:2] += np.asarray([-normal[1], normal[0]])*.75
                        normal /= max(np.linalg.norm(normal), 1e-12)
                        neighbours.append((when, normal*scale, neighbour, threshold))
                    _, obstacle_hits = self._obstacle_collisions({name: forecasts[name]}, states)
                    for _, obstacle, when in obstacle_hits:
                        if when <= 1e-6:
                            continue
                        center, scale = self._obstacle_geometry(obstacle, states[name], when)
                        relative = (_at(forecasts[name], when, self.dt)-center)*scale
                        if np.linalg.norm(relative) < .05:
                            relative = (positions[name]-center)*scale
                        normal = relative / max(np.linalg.norm(relative), 1e-12)
                        neighbours.append((when, normal*scale, center, 1.001))
                    constraints[name].extend(neighbours)
                    diagnostics['collision_constraints'] += len(neighbours)
                    if not neighbours:
                        continue
                    forecast, qp_status = self.qp.solve(positions[name], velocities[name], qp_targets[name], constraints[name], forecasts[name]['accelerations'][:-1])
                    diagnostics['qp_status'][name] = qp_status
                    if forecast is None:
                        failed = True; break
                    forecasts[name] = forecast
                if failed:
                    status = 'BRAKING'; break
            else:
                status = 'BRAKING'
        else:
            status = 'BRAKING'
        if status != 'SOLVED':
            synchronized = {name: dict(state, position=measured_positions[name], velocity=measured_velocities[name], stamp=now)
                            for name, state in states.items()}
            forecasts = self._brake(synchronized)
            braking_states = {name: dict(state, _tracking_uncertainty=0.0) for name, state in states.items()}
            minimum, collisions = self._validate(forecasts, braking_states)
            _, obstacle_collisions = self._obstacle_collisions(forecasts, braking_states)
            if collisions or obstacle_collisions or not self._bounded(forecasts, braking=True):
                status = 'UNSAFE'
                diagnostics['reason'] = 'no validated joint trajectory or collision-free braking trajectory'
        else:
            minimum, _ = self._validate(forecasts, states)
            self.goals = targets
            self.last_accelerations = {name: forecast['accelerations'][:-1].copy() for name, forecast in forecasts.items()}
        obstacle_minimum, _ = self._obstacle_collisions(forecasts, braking_states if status != 'SOLVED' else states)
        diagnostics['min_obstacle_scaled_distance'] = obstacle_minimum if math.isfinite(obstacle_minimum) else None
        diagnostics['event_resets'] = sorted(set(diagnostics['event_resets']))
        diagnostics['min_scaled_distance'] = minimum if math.isfinite(minimum) else None
        diagnostics['solve_ms'] = (wall_time.perf_counter()-started)*1000
        result = self._base(now, status, self._wire(forecasts, goals), diagnostics)
        if status in ('SOLVED', 'BRAKING'):
            self.previous = result
        return result
