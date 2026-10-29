if __name__ == "__main__":
    import runpy
    runpy.run_module('stream_listener', run_name="__main__")
else:
    import sys
    import stream_listener as _implementation
    sys.modules[__name__] = _implementation
