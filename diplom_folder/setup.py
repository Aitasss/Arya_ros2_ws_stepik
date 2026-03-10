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
        #Связываем камеру с роботом
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hrusha',
    maintainer_email='hrusha@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'tag_to_yaml = diplom_folder.tag_to_yaml:main',
            'map_to_tag_publisher = diplom_folder.map_to_tag_publisher:main',
            'map_to_base_publisher = diplom_folder.map_to_base_publisher:main',
        ],
    },
)
