from glob import glob
from setuptools import find_packages, setup


setup(
    name="cylinder_push_planner",
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/cylinder_push_planner"]),
        ("share/cylinder_push_planner", ["package.xml", "README.md"]),
        ("share/cylinder_push_planner/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="woo114515",
    maintainer_email="woo114515@users.noreply.github.com",
    description="Motion-free target selection and push geometry visualization.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "selection_planner = cylinder_push_planner.selection_node:main",
        ],
    },
)
