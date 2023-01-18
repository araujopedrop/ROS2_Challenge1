from setuptools import setup

package_name = 'turtlesim_catch_all_turtles'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ppa',
    maintainer_email='ppa@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "turtleSpawner = turtlesim_catch_all_turtles.turtleSpawner:main",
            "turtleController = turtlesim_catch_all_turtles.turtle_controller:main"
        ],
    },
)
