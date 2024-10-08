from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pleaseland'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='john',
    maintainer_email='junekyoopark@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_detect_hw = pleaseland.aruco_detect_hw:main',
            'aruco_detect_sitl = pleaseland.aruco_detect_sitl:main',
            'rl_landing = pleaseland.rl_landing:main',
        ],
    },
)
