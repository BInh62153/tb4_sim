from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'task_manager'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test', '*.tests', '*.tests.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('../../config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Iris',
    maintainer_email='iris@hoasen.edu.vn',
    description='TurtleBot4 Task Automation — Event-driven, modular architecture',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task_manager_lifecycle = task_manager.lifecycle_node:main',
        ],
    },
)
