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
    zip_safe=False,
    maintainer='dongxu',
    maintainer_email='dongxu@todo.todo',
    description='SUTD workspace',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
                'keypressed_talker = sutd_ws.keypressed_talker:main',
                'keypressed_listener = sutd_ws.keypressed_listener:main',
                'keypressed_calculator = sutd_ws.keypressed_calculator:main',
                'autonomous_driving1 = sutd_ws.autonomous_driving1:main',
                'autonomous_driving2 = sutd_ws.autonomous_driving2:main',
                'autonomous_driving3=sutd_ws.autonomous_driving3:main',
                'gap_follower = sutd_ws.gap_follower:main',
                'gap_follower2 = sutd_ws.gap_follower2:main',
                'gap_follower3 = sutd_ws.gap_follower3:main',
                'wall_follower = sutd_ws.wall_follower:main',
                'wall_follower2 = sutd_ws.wall_follower2:main',
        ],
    },
)
