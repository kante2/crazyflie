"""PBS의 부분 우선순위 분기를 응용한 4초 속도 계획 탐색.

https://arxiv.org/abs/1812.06356
공간 경로를 탐색하는 전체 PBS가 아니다. 이미 생성된 경로의 속도 후보를
검사하고, 진행 가능한 분기의 우선순위만 확정한다. 최적성/완전성은 보장하지 않는다.
"""

import itertools


def precedes(edges, first, last):
    """winner -> loser 그래프의 간접 선행 관계까지 확인한다."""
    pending, seen = [first], set()
    while pending:
        robot = pending.pop()
        if robot == last:
            return True
        if robot not in seen:
            seen.add(robot)
            pending.extend(b for a, b in edges if a == robot)
    return False


def topological_order(robots, edges, rank):
    remaining, ordered = set(robots), []
    while remaining:
        ready = [
            r for r in remaining
            if not any(b == r and a in remaining for a, b in edges)
        ]
        if not ready:
            raise ValueError("순환 우선순위는 실행할 수 없습니다.")
        robot = min(ready, key=rank)
        ordered.append(robot)
        remaining.remove(robot)
    return ordered


class PrioritySearch:
    """임박한 충돌부터 양방향 우선순위를 분기하고 불가능한 분기를 되돌린다."""

    def __init__(self, limit):
        self.limit = limit

    def solve(
        self, robots, conflicts, committed, choices, evaluate, moving, preference, rank,
    ):
        conflicts = sorted(conflicts, key=lambda p: (p["ttc"], tuple(p["robots"])))
        fixed = frozenset(committed)
        expanded = backtracks = 0
        exhausted = False
        partial, partial_score = None, None
        visited = set()

        # 무관한 드론의 이동으로 교착된 충돌 그룹을 진행 가능하다고 판정하지 않는다.
        links = fixed | frozenset(tuple(p["robots"]) for p in conflicts)
        groups, unseen = [], {r for edge in links for r in edge}
        while unseen:
            group, pending = set(), [min(unseen)]
            while pending:
                robot = pending.pop()
                if robot in group:
                    continue
                group.add(robot)
                pending.extend(
                    b if a == robot else a for a, b in links if robot in (a, b)
                )
            unseen -= group
            if any(max(choices[r]) > 0 for r in group):
                groups.append(group)

        def search(index, edges, decisions):
            nonlocal expanded, backtracks, exhausted, partial, partial_score
            key = (index, edges)
            if key in visited:
                return None
            if expanded >= self.limit:
                exhausted = True
                return None
            visited.add(key)
            expanded += 1
            if index < len(conflicts):
                pair = conflicts[index]
                preferred, other, reason = preference(pair)
                directions = [
                    (preferred, other, reason), (other, preferred, "feasible_reverse"),
                ]
                for winner, loser, why in directions:
                    if precedes(edges, loser, winner):
                        backtracks += 1
                        continue
                    # 같은 쌍의 확정 예약은 재선택하거나 나이로 뒤집지 않는다.
                    addition = {} if (winner, loser) in fixed else {
                        tuple(pair["robots"]): {
                            "winner": winner, "loser": loser,
                            "priority_reason": why, "preferred": preferred,
                        }
                    }
                    result = search(
                        index + 1, edges | {(winner, loser)}, {**decisions, **addition},
                    )
                    if result is not None:
                        return result
                    backtracks += 1
                    if exhausted:
                        break
                return None

            order = topological_order(robots, edges, rank)
            yielding = {b for _, b in edges}
            # 이미 확정한 그룹의 우선 기체가 구역/재계획을 기다리는 동안에는
            # 독립 그룹의 진행을 막지 않는다. 새 예약은 진행을 확인해야 한다.
            required = [
                g for g in groups
                if any(r not in yielding and max(choices[r]) > 0 for r in g)
                or any(d["winner"] in g for d in decisions.values())
            ]
            options = [[0.0] if r in yielding else choices[r] for r in order]
            for values in itertools.product(*options):
                candidate = dict(zip(order, values))
                safe, pairs = evaluate(candidate)
                if not safe:
                    continue
                blocked = [
                    g for g in required
                    if not any(moving(r, candidate[r]) for r in g)
                ]
                if not blocked:
                    return candidate, pairs, edges, decisions, order
                # 해결 불가능한 그룹의 임시 순서는 버리고 독립적인 그룹만 진행한다.
                # 이 후보도 정지 그룹과 이동 그룹을 포함해 모든 쌍의 검사를 통과했다.
                blocked_robots = set().union(*blocked)
                if any(candidate[r] > 0 for r in blocked_robots):
                    continue
                accepted = {
                    k: d for k, d in decisions.items()
                    if d["winner"] not in blocked_robots
                }
                accepted_edges = fixed | {
                    (d["winner"], d["loser"]) for d in accepted.values()
                }
                score = (
                    len(required) - len(blocked),
                    sum(moving(r, candidate[r]) for r in robots),
                    sum(candidate.values()),
                )
                if partial_score is None or score > partial_score:
                    partial_score = score
                    partial = (
                        candidate, pairs, accepted_edges, accepted,
                        topological_order(robots, accepted_edges, rank), sorted(blocked_robots),
                    )
            return None

        found = search(0, fixed, {})
        blocked_robots = []
        if found is None:
            if partial is not None and not exhausted:
                selected, pairs, edges, decisions, order, blocked_robots = partial
                status = "PARTIAL" if partial_score[0] > 0 else "BLOCKED"
            else:
                selected = {r: 0.0 for r in robots}
                _, pairs = evaluate(selected)
                edges, decisions = fixed, {}
                order = topological_order(robots, fixed, rank)
                status = "SEARCH_LIMIT" if exhausted else "BLOCKED"
                blocked_robots = sorted(r for r in robots if max(choices[r]) > 0)
        else:
            selected, pairs, edges, decisions, order = found
            status = "SOLVED"
        return selected, pairs, decisions, {
            "algorithm": "PBS_WINDOW", "status": status,
            "expanded": expanded, "backtracks": backtracks,
            "priority_edges": [list(edge) for edge in sorted(edges)], "order": order,
            "blocked_robots": blocked_robots,
        }
