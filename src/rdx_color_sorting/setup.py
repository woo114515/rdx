from glob import glob
from setuptools import find_packages, setup


package_name = "rdx_color_sorting"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="woo114515",
    maintainer_email="woo114515@users.noreply.github.com",
    description="Three-color perception and sorting controller for RDX.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "color_sorting_node = rdx_color_sorting.node:main",
            "safety_arbiter_node = rdx_color_sorting.safety_node:main",
        ],
    },
)
