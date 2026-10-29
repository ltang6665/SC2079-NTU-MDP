if __name__ == "__main__":
    import runpy
    runpy.run_module('communications.pc_link', run_name="__main__")
else:
    import sys
    import communications.pc_link as _implementation
    sys.modules[__name__] = _implementation
