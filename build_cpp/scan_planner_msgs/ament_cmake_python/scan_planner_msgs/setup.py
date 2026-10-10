from setuptools import find_packages
from setuptools import setup

setup(
    name='scan_planner_msgs',
    version='0.1.0',
    packages=find_packages(
        include=('scan_planner_msgs', 'scan_planner_msgs.*')),
)
