from setuptools import find_packages, setup
from glob       import glob
import os

package_name = 'perception_and_controller'


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name,'Perception','Perception.model','Perception.utils'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='emav',
    maintainer_email='emav@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'controller = perception_and_controller.controller:main',
        'controllerTest = perception_and_controller.controller_test:main',
        'perception = perception_and_controller.perception:main',
        'real_time_plotter = perception_and_controller.real_time_plotter:main',
        ],
    },
)
