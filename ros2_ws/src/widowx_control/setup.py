from setuptools import setup

package_name = "widowx_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch",
         ["launch/widowx_control.launch.py"]),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="Lucas Martins",
    maintainer_email="lucaspmartins14@gmail.com",
    description="Driver ArmLink e GUI de controle manual para o WidowX Mark II",
    license="MIT",
    entry_points={
        "console_scripts": [
            "widowx_driver = widowx_control.driver_node:main",
            "widowx_gui = widowx_control.gui_node:main",
        ],
    },
)
