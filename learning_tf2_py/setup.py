from setuptools import setup

package_name = 'learning_tf2_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/static_broadcaster.launch.py',
            'launch/dynamic_broadcaster.launch.py',
            'launch/turtle_listener.launch.py',
            'launch/camera_link.launch.py',
            'launch/camera_optical.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YOUR_NAME',
    maintainer_email='YOUR_EMAIL',
    description='TF2 tutorials implemented in Python without OOP.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'static_turtle_tf2_broadcaster = learning_tf2_py.static_turtle_tf2_broadcaster:main',
            'turtle_tf2_broadcaster = learning_tf2_py.turtle_tf2_broadcaster:main',
            'turtle_tf2_listener = learning_tf2_py.turtle_tf2_listener:main',
            'camera_link_broadcaster = learning_tf2_py.camera_link_broadcaster:main',
            'camera_optical_broadcaster = learning_tf2_py.camera_optical_broadcaster:main',
        ],
    },
)