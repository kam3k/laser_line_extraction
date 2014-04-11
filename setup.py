from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['laser_line_extraction'],
    package_dir={'': 'src'},
)

setup(**setup_args) 
