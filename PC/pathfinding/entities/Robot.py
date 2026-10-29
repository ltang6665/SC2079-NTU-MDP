if __name__ == "__main__":
    import runpy
    runpy.run_module('pathfinding.entities.robot_model', run_name="__main__")
else:
    import sys
    import pathfinding.entities.robot_model as _implementation
    sys.modules[__name__] = _implementation
