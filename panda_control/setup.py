from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'panda_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='scg1224',
    maintainer_email='caos1@rose-hulman.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_move = movebot.simple_move:main',
            'panda_control = panda_control.panda_control:main',
            'tag_tf = panda_control.tag_tf:main'
        ],
    },
)
