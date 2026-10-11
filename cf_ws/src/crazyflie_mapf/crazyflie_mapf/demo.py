"""Synthetic two-agent precedence example; no ROS nodes or flight commands."""

import json

from .priority import PrioritySearch


def crossing_example():
    """Use an explicit compatibility table as a toy conflict-checking oracle."""
    robots = ('cf1', 'cf2')

    def evaluate(candidate):
        # Illustrative input: either drone can move alone, but not both together.
        # Real integration must replace this with swept-trajectory validation.
        safe = not all(candidate[r] > 0.0 for r in robots)
        return safe, []

    return PrioritySearch(limit=128).solve(
        robots=robots,
        conflicts=[{'robots': list(robots), 'ttc': 1.0}],
        committed=[],
        choices={r: [1.0, 0.4, 0.0] for r in robots},
        evaluate=evaluate,
        moving=lambda robot, speed: speed > 0.0,
        preference=lambda pair: ('cf1', 'cf2', 'demo_preference'),
        rank=lambda robot: robot,
    )


def main():
    selected, _, decisions, diagnostics = crossing_example()
    print(json.dumps({
        'example': 'synthetic compatibility table; no flight commands',
        'speed_limits': selected,
        'decisions': list(decisions.values()),
        'search': diagnostics,
    }, indent=2))


if __name__ == '__main__':
    main()
