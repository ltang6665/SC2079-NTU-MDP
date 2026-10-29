if __name__ == "__main__":
    import runpy
    runpy.run_module('pathfinding.route_solver', run_name="__main__")
else:
    import sys
    import pathfinding.route_solver as _implementation
    sys.modules[__name__] = _implementation
