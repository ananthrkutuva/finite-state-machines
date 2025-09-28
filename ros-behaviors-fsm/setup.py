from setuptools import find_packages, setup

package_name = 'ros-behaviors-fsm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='akutuva',
    maintainer_email='akutuva@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'teleop = ros-behaviors-fsm.teleop:main',
            'drive_square = ros-behaviors-fsm.drive_square:main',
        ],
    },
)
