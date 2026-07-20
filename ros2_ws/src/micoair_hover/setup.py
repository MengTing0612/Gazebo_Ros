from setuptools import find_packages, setup


package_name = "micoair_hover"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/micoair_hover"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", ["config/hover_sim.yaml"]),
        (f"share/{package_name}/launch", ["launch/hover_sim.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="MicoAir Hover",
    maintainer_email="maintainer@example.invalid",
    description="PX4 offboard hover baseline and bounded SDK residual controller.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "offboard_hover = micoair_hover.offboard_hover_node:main",
        ],
    },
)
