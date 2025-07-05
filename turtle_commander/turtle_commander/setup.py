from setuptools import setup

package_name = 'turtle_commander'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/srv', ['srv/ResetTurtle.srv']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yourname',
    maintainer_email='your@email.com',
    description='Turtle commander project',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'commander_node = turtle_commander.commander_node:main',
            'reset_service_node = turtle_commander.reset_service_node:main',
        ],
    },
)
