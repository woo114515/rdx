from glob import glob

from setuptools import find_packages, setup


package_name = "simple_task3"


setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/simple_task3"]),
        ("share/simple_task3", ["package.xml", "README.md"]),
        ("share/simple_task3/config", glob("config/*.yaml")),
        ("share/simple_task3/launch", glob("launch/*.launch.py")),
        ("share/simple_task3/scripts", glob("scripts/*.sh")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="woo114515",
    maintainer_email="woo114515@users.noreply.github.com",
    description="Simple map-frame path execution with automatic Task 3 cycling.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "simple_task_controller = simple_task3.controller_node:main",
            "localization_handoff = simple_task3.localization_handoff_node:main",
        ],
    },
)
