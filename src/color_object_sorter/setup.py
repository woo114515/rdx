from glob import glob
from setuptools import find_packages, setup


package_name = "color_object_sorter"


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
    description="Multi-object color perception and tracking for sorting tasks.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "color_object_detector = color_object_sorter.detector_node:main",
            "color_sorter_debug_viewer = color_object_sorter.debug_viewer_node:main",
            "color_lidar_fusion = color_object_sorter.fusion_node:main",
            "temporal_object_confirmation = color_object_sorter.confirmation_node:main",
            "hsv_sampler = color_object_sorter.hsv_sampler_node:main",
            "hsv_analyzer = color_object_sorter.hsv_analyzer:main",
            "candidate_validator = color_object_sorter.candidate_validation_node:main",
        ],
    },
)
