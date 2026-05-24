from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_robot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'mapas'), glob('mapas/*')), #esse trem identifica o mapa (euyacho)
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rks',
    maintainer_email='ramonkaina.santos@gmail.com',
    description='Pacote de inicialização do robô e visualização de mapas',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'base_driver = serial_com_py.base_driver:main',
        'base_driver_pulse = serial_com_py.base_driver_pulse:main',
    ],
    },
)