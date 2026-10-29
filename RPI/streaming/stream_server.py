if __name__ == "__main__":
    import runpy
    runpy.run_module('streaming.camera_server', run_name="__main__")
else:
    import sys
    import streaming.camera_server as _implementation
    sys.modules[__name__] = _implementation
