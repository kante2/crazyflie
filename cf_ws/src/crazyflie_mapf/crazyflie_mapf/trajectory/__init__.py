"""Online per-drone DMPC trajectory generation, independent of ROS."""

__all__ = ["MultiDroneTrajectoryPlanner", "sample_trajectory"]


def __getattr__(name):
    """Allow pure metrics imports without importing the optional QP solver."""
    if name not in __all__:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    from importlib import import_module
    value = getattr(import_module(".functions", __name__), name)
    globals()[name] = value
    return value
