from setuptools import setup

package_name = 'ur'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your@email.com',
    description='UR package with joint_state_merger node',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_state_merger = ur.joint_state_merger:main',
            'gazebo_moveit_bridge = ur.gazebo_moveit_bridge:main'
        ],
    },
)