from setuptools import find_packages, setup

package_name = 'fp_amr_vision'

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
    maintainer='rokey',
    maintainer_email='kings0625@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'waypoint_patrol = fp_amr_vision.waypoint_patrol:main',
            'patrol_fsm = fp_amr_vision.patrol_fsm:main',
            'amr_agent = fp_amr_vision.amr_agent:main',
            'amr_patrol_emer_helmet = fp_amr_vision.amr_patrol_emer_helmet:main',
            'fleet_fsm = fp_amr_vision.fleet_fsm:main'
        ],
    },
)
