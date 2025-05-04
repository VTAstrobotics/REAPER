# setup.py
from glob import glob
from setuptools import find_packages, setup

package_name = "ukf"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        # mandatory index & package
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),

        #  ⬇ NEW: install launch and config files
        ("share/" + package_name + "/launch",  glob("launch/*.py")),
        ("share/" + package_name + "/config",  glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="astro-dev",
    maintainer_email="jeronimus.ryan@gmail.com",
    description="UKF wrapper around robot_localization",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],   # you’re not exporting a Python node
    },
)
