from glob import glob
from setuptools import find_packages, setup


setup(
    name="cylinder_push_planner",
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/cylinder_push_planner"]),
        ("share/cylinder_push_planner", ["package.xml", "README.md"]),
        ("share/cylinder_push_planner/config", glob("config/*.yaml")),
        ("share/cylinder_push_planner/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="woo114515",
    maintainer_email="woo114515@users.noreply.github.com",
    description="Compact direct Task 3 control with legacy staged tools.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "selection_planner = cylinder_push_planner.selection_node:main",
            "reobservation = cylinder_push_planner.reobservation_node:main",
            "approach_execution = cylinder_push_planner.approach_execution_node:main",
            "push_cycle_execution = cylinder_push_planner.approach_execution_node:main",
            "task_orchestrator = cylinder_push_planner.task_orchestrator_node:main",
            "direct_task_controller = "
            "cylinder_push_planner.direct_task_node:main",
        ],
    },
)
