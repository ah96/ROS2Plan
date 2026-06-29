from setuptools import setup

package_name = 'ros2plan_action_interface'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'action_interface_manager = ros2plan_action_interface.ActionInterfaceManager:main',
        ],
    },
)
