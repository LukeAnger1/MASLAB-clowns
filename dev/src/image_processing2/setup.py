from setuptools import find_packages, setup

package_name = 'image_processing2'

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
    maintainer='maslab-clown-penis',
    maintainer_email='anger.luke2004Agmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cube_detect = image_processing2.cube_detect_node:main',
            'cube_locate = image_processing2.cube_locate_node:main'
        ],
    },
)
