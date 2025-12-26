from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'hand_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.py'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hartvi',
    maintainer_email='j.hartvich@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hand_publisher_node=hand_publisher_node.hand_publisher_node:main',
            'hand_image_node=hand_publisher_node.hand_image_node:main',
            'hand_points_node=hand_publisher_node.hand_points_node:main',
        ],
    },
)
