from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'diplom_folder'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
          # Добавляем launch файлы
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Добавляем конфиги
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hrusha',
    maintainer_email='hrusha@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
        ],
    },
)
