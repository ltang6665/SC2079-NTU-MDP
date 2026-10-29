if __name__ == "__main__":
    import runpy
    runpy.run_module('image_stitching', run_name="__main__")
else:
    import sys
    import image_stitching as _implementation
    sys.modules[__name__] = _implementation
