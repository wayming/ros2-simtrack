from setuptools import setup
from glob import glob

package_name = 'turtle_commander'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rosidl_runtime_py'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='user@email.com',
    description='Turtle commander Python node',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'commander_node = turtle_commander.commander_node:main',
            'reset_service = turtle_commander.reset_service:main',
        ],
    },
)
