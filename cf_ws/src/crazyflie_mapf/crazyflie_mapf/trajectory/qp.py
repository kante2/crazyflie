"""Condensed double-integrator QP used separately by each DMPC agent.

This is a practical adaptation of Luis et al. (RA-L 2020, arXiv:1909.05150):
acceleration samples replace the paper's Bezier position-reference inputs and
identified tracking dynamics. Dynamics are exact between constant-acceleration
knots; speed/actuation constraints use a conservative L1 polytope, so their
Euclidean norms cannot exceed the configured limits. No global fleet QP is built.
"""

from itertools import product

import numpy as np
import osqp
from scipy import sparse


class AgentQP:
    """A single agent's inputs with other forecasts treated as communicated data."""

    def __init__(self, steps, dt, config):
        self.steps, self.dt, self.config = steps, dt, config
        self.dimension = 3 * steps
        self.position = np.zeros((steps + 1, 3, self.dimension))
        self.velocity = np.zeros_like(self.position)
        for k in range(1, steps + 1):
            for j in range(k):
                self.position[k, :, 3*j:3*j+3] = np.eye(3) * dt**2 * (k-j-.5)
                self.velocity[k, :, 3*j:3*j+3] = np.eye(3) * dt
        p_matrix = self.position[1:].reshape(-1, self.dimension)
        # Progress throughout the window, with increased emphasis near its end.
        weights = np.repeat(np.linspace(2.0, 8.0, steps), 3)
        self.goal_matrix = p_matrix
        self.goal_weights = weights
        difference = np.eye(self.dimension)
        for k in range(1, steps):
            difference[3*k:3*k+3, 3*k-3:3*k] -= np.eye(3)
        self.hessian = sparse.csc_matrix(2 * (
            p_matrix.T @ (weights[:, None] * p_matrix)
            + np.eye(self.dimension) * .12
            + difference.T @ difference * .08
        ))
        # All eight signed sums <= bound implement ||vector||_1 <= bound.
        self.signs = np.asarray(list(product((-1.0, 1.0), repeat=3)))
        acceleration = np.kron(np.eye(steps), self.signs)
        velocity = np.vstack([self.signs @ self.velocity[k] for k in range(1, steps+1)])
        self.base_matrix = np.vstack((p_matrix, velocity, acceleration, self.velocity[-1]))
        self.position_rows = len(p_matrix)
        self.velocity_rows = len(velocity)
        self.acceleration_rows = len(acceleration)

    def at(self, time):
        """Map acceleration decisions to position at any point in the window."""
        k = min(self.steps - 1, max(0, int(time / self.dt)))
        within = min(self.dt, max(0.0, time - k*self.dt))
        matrix = self.position[k] + within*self.velocity[k]
        matrix = matrix.copy()
        matrix[:, 3*k:3*k+3] += np.eye(3) * (.5*within**2)
        return matrix

    def solve(self, position, velocity, goal, constraints=(), initial=None):
        times = np.arange(1, self.steps+1)[:, None] * self.dt
        free_positions = position + times*velocity
        target_error = (free_positions - goal).reshape(-1)
        linear = 2*self.goal_matrix.T @ (self.goal_weights*target_error)
        low, high = np.asarray(self.config['bounds'])
        p_low = np.tile(low, self.steps) - free_positions.reshape(-1)
        p_high = np.tile(high, self.steps) - free_positions.reshape(-1)
        signed_velocity = np.tile(self.signs @ velocity, self.steps)
        v_high = self.config['speed'] - signed_velocity
        a_high = np.full(self.acceleration_rows, self.config['acceleration'])
        lower = np.concatenate((p_low, np.full(self.velocity_rows+self.acceleration_rows, -np.inf), -velocity))
        upper = np.concatenate((p_high, v_high, a_high, -velocity))
        matrix = self.base_matrix
        if constraints:
            rows, bounds = [], []
            for time, normal, neighbour, clearance in constraints:
                rows.append(normal @ self.at(time))
                bounds.append(clearance + normal @ (neighbour-position-time*velocity))
            matrix = np.vstack((matrix, rows))
            lower = np.concatenate((lower, bounds))
            upper = np.concatenate((upper, np.full(len(bounds), np.inf)))
        solver = osqp.OSQP()
        solver.setup(P=self.hessian, q=linear, A=sparse.csc_matrix(matrix),
                     l=lower, u=upper, verbose=False, eps_abs=1e-5,
                     eps_rel=1e-5, max_iter=4000, polishing=True,
                     warm_starting=True)
        if initial is not None:
            solver.warm_start(x=np.asarray(initial).reshape(-1))
        result = solver.solve(raise_error=False)
        if result.info.status_val not in (1, 2) or result.x is None:
            return None, result.info.status
        inputs = result.x.reshape(self.steps, 3)
        positions = position + np.arange(self.steps+1)[:, None]*self.dt*velocity + np.einsum('kij,j->ki', self.position, result.x)
        velocities = velocity + np.einsum('kij,j->ki', self.velocity, result.x)
        return {
            'positions': positions,
            'velocities': velocities,
            'accelerations': np.vstack((inputs, np.zeros(3))),
        }, result.info.status
