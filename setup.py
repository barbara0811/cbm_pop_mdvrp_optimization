## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['cbm_pop_lib',
	      'cbm_pop_lib.modules',
              'cbm_pop_lib.operators',
              'cbm_pop_lib.common',],
    package_dir={'': 'src'})

setup(**setup_args)
