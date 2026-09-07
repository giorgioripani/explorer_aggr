import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'explorer_aggr'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # --- RIGA AGGIUNTA PER INSTALLARE I FILE YAML ---
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='giorgio',
    maintainer_email='giorgio.ripani@outlook.com',
    description='Pacchetto generato da AG e GR per esplorazione autonoma del robot',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'communication_node = explorer_aggr.communication_node:main',
        ],
    },
)