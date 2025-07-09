from setuptools import setup, find_packages

package_name = 'dfrobot_imu_ahrs_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'numpy',  # Required for Madgwick filter
    ],
    zip_safe=True,
    maintainer='pi4',
    maintainer_email='pi4@todo.todo',
    description='ROS 2 package for BMX160 IMU interfacing using DFRobot library and sensor fusion',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bmx160_imu_ahrs_node = dfrobot_imu_ahrs_driver.bmx160_imu_ahrs_node:main',
        ],
    },
)
