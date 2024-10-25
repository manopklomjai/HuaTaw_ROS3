from setuptools import setup, find_packages

package_name = 'imu'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    install_requires=['setuptools'],
    package_data={
        # No need to include msg/*.msg here
    },
    zip_safe=True,
    maintainer='orin_nano',
    maintainer_email='orin_nano@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bmi160 = imu.bmi160:main'
        ],
    },
)
