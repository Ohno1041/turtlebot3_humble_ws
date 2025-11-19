from setuptools import find_packages, setup

package_name = 'my_continuous_nav'

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
    maintainer='root',
    maintainer_email='astralshine3.14@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'continuous_navigator = my_continuous_nav.continuous_navigator:main',
            'pose_subscriber = my_continuous_nav.pose_subscriber:main',
            'spawn_obstacle = my_continuous_nav.spawn_obstacle:main',
            'send_goal = my_continuous_nav.send_goal:main',
        ],
    },
)
