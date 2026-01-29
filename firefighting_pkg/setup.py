from setuptools import find_packages, setup

package_name = 'firefighting_pkg'

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
    maintainer='fariza',
    maintainer_email='ri06nuha@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sim_yolo_node = firefighting_pkg.sim_yolo_node:main',
            'sim_depth_fire_node = firefighting_pkg.sim_depth_fire_node:main',
            'yolo_node = firefighting_pkg.yolo_node:main',
            'mavlink_controller_node = firefighting_pkg.mavlink_controller_node:main',
            'working_mavlink__node = firefighting_pkg.mavlink_works:main',
            'mavros_controller_node = firefighting_pkg.mavros_controller_node:main',
            'fire_vision_node = firefighting_pkg.fire_vision_node:main',
            'mission_manager = firefighting_pkg.mission_manager:main',
            'try_mavros = firefighting_pkg.try_mavros:main',
        ],
    },
)
