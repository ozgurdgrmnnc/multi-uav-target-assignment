from glob import glob

from setuptools import find_packages, setup

package_name = "iha_wta"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Özgür",
    maintainer_email="ozgurdgrmnnc@gmail.com",
    description="Dağıtık çoklu İHA hedef atama",
    license="MIT",
    entry_points={
        "console_scripts": [
            "world_node = iha_wta.world_node:main",
            "uav_agent_node = iha_wta.uav_agent_node:main",
            "gz_viz_node = iha_wta.gz_viz_node:main",
        ],
    },
)
