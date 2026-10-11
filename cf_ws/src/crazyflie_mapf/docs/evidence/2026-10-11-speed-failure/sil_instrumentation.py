import sys
if 'trajectory_planner' in sys.argv[0]:
    import json
    import numpy as np
    from pathlib import Path
    from crazyflie_mapf.trajectory.functions import MultiDroneTrajectoryPlanner as Planner
    path = Path('/tmp/mapf_speed_diagnosis')
    original_plan, original_bounded = Planner.plan, Planner._bounded
    original_validate, original_obstacles = Planner._validate, Planner._obstacle_collisions
    def bounded(self, forecasts, braking=False):
        result = original_bounded(self, forecasts, braking)
        if not result:
            self.audit['bounds_failures'].append(dict(braking=braking,
                robots={n:dict(min_pos=f['positions'].min(axis=0).tolist(), max_pos=f['positions'].max(axis=0).tolist(),
                               max_speed=float(np.linalg.norm(f['velocities'],axis=1).max()),
                               max_accel=float(np.linalg.norm(f['accelerations'],axis=1).max()),
                               terminal_speed=float(np.linalg.norm(f['velocities'][-1]))) for n,f in forecasts.items()}))
        return result
    def validate(self, forecasts, states, threshold=1.):
        result = original_validate(self, forecasts, states, threshold)
        self.audit['last_pair_check'] = dict(minimum=result[0] if np.isfinite(result[0]) else None, collisions=result[1], threshold=threshold)
        return result
    def obstacles(self, forecasts, states):
        result = original_obstacles(self, forecasts, states)
        self.audit['last_obstacle_check'] = dict(minimum=result[0] if np.isfinite(result[0]) else None,
            collisions=[(n,o['name'],t) for n,o,t in result[1]])
        return result
    def plan(self, states, goals, now):
        self.audit = dict(bounds_failures=[])
        b = original_plan(self, states, goals, now)
        summary = dict(stamp=now,status=b['status'],diagnostics=b['diagnostics'],audit=self.audit,
                       positions={n:s['position'] for n,s in states.items()},velocities={n:s['velocity'] for n,s in states.items()})
        with (path/'trace.jsonl').open('a') as f:f.write(json.dumps(summary)+'\n')
        if b['status']=='UNSAFE' and not (path/'first_unsafe.json').exists():
            (path/'first_unsafe.json').write_text(json.dumps(dict(states=states,goals=goals,batch=b,audit=self.audit)))
        return b
    Planner.plan,Planner._bounded,Planner._validate,Planner._obstacle_collisions = plan,bounded,validate,obstacles
    (path/'instrumentation_ready').write_text(str(sys.argv))
