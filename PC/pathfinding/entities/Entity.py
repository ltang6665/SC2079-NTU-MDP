if __name__ == "__main__":
    import runpy
    runpy.run_module('pathfinding.entities.arena', run_name="__main__")
else:
    import sys
    import pathfinding.entities.arena as _implementation
    sys.modules[__name__] = _implementation
