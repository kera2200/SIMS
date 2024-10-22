from setuptools import find_packages, setup


package_name = 'first_package'

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
    maintainer='turtle',
    maintainer_email='turtle@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlebot4_first_python_node = first_package.turtlebot4_first_python_node:main',
            'turtlebot4_forward_node = first_package.turtlebot4_forward_node:main',
            'turtlebot4_scan_test_node = first_package.turtlebot4_scan_test_node:main',
            'turtlebot4_display_test_node = first_package.turtlebot4_display_test_node:main',
            'turtlebot4_frontiers_explore_test_node = first_package.turtlebot4_frontiers_explore_test_node:main',
            'turtlebot_frontier_explore = first_package.turtlebot_frontier_explore:main',
            'turtlebot4_map_nav = first_package.turtlebot4_map_nav:main'
            'Turtlebot4_Frontier_Explorer = first_package.Turtlebot4_Frontier_Explorer:main',
            
        ],
    },
)
