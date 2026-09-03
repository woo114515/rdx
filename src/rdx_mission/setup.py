from setuptools import find_packages, setup


PACKAGE_NAME = "rdx_mission"


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
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="RDX Team",
    maintainer_email="maintainers@example.com",
    description="Fixed-order NavigateToPose mission for task two.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "rdx_mission_node = rdx_mission.mission_node:main",
        ],
    },
)
