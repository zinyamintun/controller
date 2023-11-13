from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['linearization', 'human_roadmap'],
    # scripts=['src/latch.py', 'src/predictor_ros.py', 'src/webcam_node.py', 'src/smooth.py', 'src/safety_switch.py', 'src/subsciber_test2.py'],
    package_dir={'': 'src/jackal'},
)


setup(**d)
