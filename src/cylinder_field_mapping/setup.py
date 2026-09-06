from glob import glob
from setuptools import find_packages, setup


setup(
    name="cylinder_field_mapping",
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/cylinder_field_mapping"],
        ),
        ("share/cylinder_field_mapping", ["package.xml", "README.md"]),
        ("share/cylinder_field_mapping/config", glob("config/*.yaml")),
        ("share/cylinder_field_mapping/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="woo114515",
    maintainer_email="woo114515@users.noreply.github.com",
    description="LiDAR-first cylinder snapshots and field geometry.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "field_mapper = cylinder_field_mapping.node:main",
            "snapshot_builder = cylinder_field_mapping.snapshot_node:main",
        ],
    },
)
