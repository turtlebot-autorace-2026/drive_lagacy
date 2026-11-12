from setuptools import find_packages, setup

package_name = 'core'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/core.launch.py', 'launch/simulation.launch.py']),
        ('share/' + package_name + '/launch', ['launch/cameras.launch.py']),
        ('share/' + package_name + '/launch', ['launch/missions.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='asilia',
    maintainer_email='keummingi0131@gmail.com',
    description='Core package for controlling turtlebot3_autorace missions',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'core = core.core:main',
        ],
    },
)
