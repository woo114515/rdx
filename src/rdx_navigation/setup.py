from glob import glob
import os

from setuptools import find_packages, setup


PACKAGE_NAME = "rdx_navigation"


setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{PACKAGE_NAME}"],
        ),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
        (os.path.join("share", PACKAGE_NAME, "config"), glob("config/*.yaml")),
        (
            os.path.join("share", PACKAGE_NAME, "launch"),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join("share", PACKAGE_NAME, "behavior_trees"),
            glob("behavior_trees/*.xml"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="RDX Team",
    maintainer_email="maintainers@example.com",
    description="SLAM mapping and static-map Nav2 launches for the RDX robot.",
    license="Apache-2.0",
)
