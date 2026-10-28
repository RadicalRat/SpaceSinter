from setuptools import setup

package_name = "srrl_irradiance"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Cameron Hinkle",
    maintainer_email="cameronhinkle@mines.edu",
    description="Publishes NREL SRRL irradiance (DNI/GHI/DHI) once a minute.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["srrl_irradiance_node = srrl_irradiance.srrl_irradiance_node:main"]},
)
