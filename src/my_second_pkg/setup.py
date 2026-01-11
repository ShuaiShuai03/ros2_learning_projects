from setuptools import find_packages, setup

package_name = 'my_second_pkg'

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
    description='Publisher and Subscriber example package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ticker = my_second_pkg.ticker_node:main',
            'listener = my_second_pkg.listener_node:main',
        ],
    },
)
