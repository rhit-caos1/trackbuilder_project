from setuptools import find_packages, setup

package_name = 'track_manager'

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
    maintainer='shane',
    maintainer_email='shanedeng@cmu.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'printer = track_manager.printer:main',
            'track_gen = track_manager.track_gen:main',
            'state_machine = track_manager.state_machine:main',
            'sm_test = track_manager.sm_test:main',
        ],
    },
)
