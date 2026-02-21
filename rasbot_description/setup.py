from setuptools import find_packages, setup
import os

package_name = 'rasbot_description'

def package_files(directory_list):
    paths = []
    for directory in directory_list:
        for path, _, filenames in os.walk(directory):
            for filename in filenames:
                file_path = os.path.join(path, filename)
                install_path = os.path.join('share', package_name, path)
                paths.append((install_path, [file_path]))
    return paths

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

data_files.extend(package_files(['launch', 'urdf', 'meshes']))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gabriel',
    maintainer_email='gabrieldiaz@hotmail.com',
    description='Robot description package for URDF and xacro examples.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)