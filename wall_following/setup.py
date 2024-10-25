from setuptools import setup

package_name = 'wall_following'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],  # This refers to the wall_following directory
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='orin_nano',
    maintainer_email='yoodyui@yahoo.com',
    description='Wall following PID node for F1tenth robot.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wall_following_pid = wall_following.wall_following_pid:main',
            'lidar_visualizer = wall_following.lidar_visualizer:main',
        ],
    },
)
