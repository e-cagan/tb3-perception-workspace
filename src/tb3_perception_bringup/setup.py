from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'tb3_perception_bringup'


def get_data_files_from_dir(source_dir, install_dir):
    """Recursively collect all files in a directory."""
    data_files = []
    for root, dirs, files in os.walk(source_dir):
        if files:
            rel_path = os.path.relpath(root, source_dir)
            install_path = os.path.join(install_dir, rel_path) if rel_path != '.' else install_dir
            file_paths = [os.path.join(root, f) for f in files]
            data_files.append((install_path, file_paths))
    return data_files


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Additional directories
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        *get_data_files_from_dir('models', os.path.join('share', package_name, 'models')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cagan',
    maintainer_email='emincaganapaydin@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_node = tb3_perception_bringup.teleop_node:main',
        ],
    },
)
