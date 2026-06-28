from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'task_manager'

# Tính đường dẫn tuyệt đối đến thư mục config từ vị trí của setup.py,
# tránh phụ thuộc vào cwd của colcon (colcon không chạy từ src/task_manager/).
_HERE = os.path.dirname(os.path.abspath(__file__))
_CONFIG_DIR = os.path.join(_HERE, '..', '..', 'config')
_config_yamls = glob(os.path.join(_CONFIG_DIR, '*.yaml'))

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test', '*.tests', '*.tests.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), _config_yamls),
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