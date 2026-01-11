from setuptools import find_packages, setup

package_name = 'my_first_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='My first ROS 2 package - Hello World node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hello = my_first_pkg.hello_node:main',
            'hello_class = my_first_pkg.hello_node_class:main',
        ],
    },
)
