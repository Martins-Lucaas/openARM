from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'fruit_sorting_webots'

# Incluir arquivos da pasta worlds/obj recursivamente
obj_files = [
    (os.path.join('share', package_name, 'worlds', 'obj', relpath), [os.path.join(dirpath, f)])
    for dirpath, _, files in os.walk('worlds/obj')
    for f in files
    for relpath in [os.path.relpath(dirpath, 'worlds/obj')]
]

# Incluir arquivos da pasta controllers/fruit_sorting_ctrl_opencv recursivamente (sem sounds)
ctrl_files = [
    (os.path.join('share', package_name, 'controllers', 'fruit_sorting_ctrl_opencv', relpath), [os.path.join(dirpath, f)])
    for dirpath, _, files in os.walk('controllers/fruit_sorting_ctrl_opencv')
    if 'sounds' not in dirpath  # <--- IGNORA "sounds"
    for f in files
    for relpath in [os.path.relpath(dirpath, 'controllers/fruit_sorting_ctrl_opencv')]
]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ROS index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Launch e mundos
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.wbt')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.jpg')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.wbproj')),
    ] + obj_files + ctrl_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lucas-pc',
    maintainer_email='lucaspmartins14@gmail.com',
    description='Sistema de triagem de frutas com Webots e ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
