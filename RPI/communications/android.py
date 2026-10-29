if __name__ == "__main__":
    import runpy
    runpy.run_module('communications.android_link', run_name="__main__")
else:
    import sys
    import communications.android_link as _implementation
    sys.modules[__name__] = _implementation
