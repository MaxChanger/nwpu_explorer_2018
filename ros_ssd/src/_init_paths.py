"""Set up paths for ros-ssd."""

import os
import os.path as osp
import sys

def add_path(path):
    if path not in sys.path:
        sys.path.insert(0, path)

lib_dir = osp.join(osp.dirname(__file__), '..', 'lib')

# Add ssd to search path
ssd_path = osp.join(lib_dir, 'SSD-Tensorflow')
add_path(ssd_path)
