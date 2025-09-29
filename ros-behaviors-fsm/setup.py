from setuptools import find_packages, setup

package_name = "ros_behaviors_fsm"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="akutuva, clinehan",
    maintainer_email="akutava@olin.edu, clinehan@olin.edu",
    description="Code for Neato based finite state controller",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "teleop = ros_behaviors_fsm.teleop:main",
            "drive_square = ros_behaviors_fsm.drive_square:main",
            "person_following = ros_behaviors_fsm.person_following:main",
            "finite_state_controller = ros_behaviors_fsm.finite_state_controller:main",
        ],
    },
)
