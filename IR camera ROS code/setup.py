from setuptools import setup
from glob import glob

package_name = 'mlx90640_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add launch files if you have any
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='grant',
    maintainer_email='grant@example.com',
    description='ROS2 node for the MLX90640 thermal camera',
    license='MIT',
    # Removed tests_require and added extras_require
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'mlx90640_serial = mlx90640_ros2.mlx90640_serial:main',
            'thermal_visualizer = mlx90640_ros2.thermal_visualizer:main',
        ],
    },
)