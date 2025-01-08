import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'fizzbuzz'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MASLAB 2024 Staff',
    maintainer_email='maslab-2024-staff@mit.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'number_publisher = fizzbuzz.number_publisher_node:main',
            'fizzbuzz_subscriber = fizzbuzz.fizzbuzz_subscriber_node:main'
        ],
    },
)
