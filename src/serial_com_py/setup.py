from setuptools import find_packages, setup

package_name = 'serial_com_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rks',
    maintainer_email='rks@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'base_driver = serial_com_py.base_driver:main',
            'safe_stop = serial_com_py.safe_stop:main',
            'base_driver_sim = serial_com_py.base_driver_sim:main', #simulador pra ver se o trem funciona sem precisar do robo
        ],
    },
)
