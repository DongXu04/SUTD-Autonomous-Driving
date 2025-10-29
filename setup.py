from setuptools import find_packages, setup

package_name = 'sutd_ws'

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
    maintainer='tan',
    maintainer_email='dongxu1104@gmail.com',
    description='sutd workspace',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'wall_follower = sutd_ws.wall_follower:main',
            'keypressed_talker = sutd_ws.keypressed_talker:main',
            'keypressed_calculator = sutd_ws.keypressed_calculator:main',
            'remote_control = sutd_ws.remote_control:main',
            'autonomous_driving = sutd_ws.autonomous_driving:main',
            'safety_driving = sutd_ws.safety_driving:main',
            'gap_follower = sutd_ws.gap_follower:main',
            'gap_follower2 = sutd_ws.gap_follower2:main',
        ],
    },
)
