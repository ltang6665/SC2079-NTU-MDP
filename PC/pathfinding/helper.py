if __name__ == "__main__":
    import runpy
    runpy.run_module('pathfinding.motion_commands', run_name="__main__")
else:
    import sys
    import pathfinding.motion_commands as _implementation
    sys.modules[__name__] = _implementation
