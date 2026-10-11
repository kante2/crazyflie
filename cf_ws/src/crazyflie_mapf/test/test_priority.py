"""Regression cases for the standalone priority-search contract."""

import pytest

from crazyflie_mapf import PrioritySearch, precedes, topological_order
from crazyflie_mapf.demo import crossing_example


def search(robots, conflicts, safe, limit=128, committed=()):
    return PrioritySearch(limit).solve(
        robots=robots,
        conflicts=[{'robots': [a, b], 'ttc': i + 1.0}
                   for i, (a, b) in enumerate(conflicts)],
        committed=committed,
        choices={r: [1.0, 0.4, 0.0] for r in robots},
        evaluate=lambda candidate: (safe(candidate), []),
        moving=lambda robot, speed: speed > 0.0,
        preference=lambda pair: (*pair['robots'], 'test_preference'),
        rank=lambda robot: robot,
    )


def test_demo_selects_one_agent_and_keeps_the_other_waiting():
    selected, _, decisions, report = crossing_example()
    assert selected == {'cf1': 1.0, 'cf2': 0.0}
    assert report['status'] == 'SOLVED'
    assert decisions[('cf1', 'cf2')]['winner'] == 'cf1'


def test_infeasible_preference_backtracks_to_reverse_order():
    selected, _, decisions, report = search(
        ('a', 'b'), [('a', 'b')], lambda candidate: candidate['a'] == 0.0,
    )
    assert selected['a'] == 0.0 and selected['b'] > 0.0
    assert decisions[('a', 'b')]['winner'] == 'b'
    assert report['status'] == 'SOLVED'


def test_safe_all_stop_is_not_a_solved_passage():
    selected, _, decisions, report = search(
        ('a', 'b'), [('a', 'b')], lambda candidate: not any(candidate.values()),
    )
    assert selected == {'a': 0.0, 'b': 0.0}
    assert decisions == {}
    assert report['status'] == 'BLOCKED'


def test_search_limit_does_not_commit_tentative_priorities():
    selected, _, decisions, report = search(
        ('a', 'b'), [('a', 'b')], lambda candidate: True, limit=1,
    )
    assert not any(selected.values())
    assert decisions == {}
    assert report['status'] == 'SEARCH_LIMIT'


def test_committed_order_cannot_be_reversed_by_preference():
    selected, _, decisions, report = search(
        ('a', 'b'), [('a', 'b')], lambda candidate: True,
        committed=[('b', 'a')],
    )
    assert selected['a'] == 0.0 and selected['b'] > 0.0
    assert decisions == {}
    assert report['priority_edges'] == [['b', 'a']]


def test_cyclic_preferences_result_in_acyclic_priorities():
    selected, _, _, report = search(
        ('a', 'b', 'c'), [('a', 'b'), ('b', 'c'), ('c', 'a')],
        lambda candidate: sum(v > 0.0 for v in candidate.values()) <= 1,
    )
    assert report['status'] == 'SOLVED'
    assert any(selected.values())
    order = report['order']
    assert all(order.index(a) < order.index(b) for a, b in report['priority_edges'])


def test_independent_group_can_progress_when_another_is_blocked():
    selected, _, decisions, report = search(
        ('a', 'b', 'c', 'd'), [('a', 'b'), ('c', 'd')],
        lambda c: c['a'] == c['b'] == 0.0 and not (c['c'] > 0 and c['d'] > 0),
    )
    assert report['status'] == 'PARTIAL'
    assert selected['a'] == selected['b'] == 0.0
    assert selected['c'] > 0.0 and selected['d'] == 0.0
    assert set(decisions) == {('c', 'd')}


def test_transitive_order_and_cycle_rejection():
    edges = [('a', 'b'), ('b', 'c')]
    assert precedes(edges, 'a', 'c')
    assert not precedes(edges, 'c', 'a')
    with pytest.raises(ValueError, match='순환'):
        topological_order(('a', 'b', 'c'), edges + [('c', 'a')], lambda r: r)
