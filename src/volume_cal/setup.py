from setuptools import setup
import os

package_name = 'volume_cal'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
        (os.path.join('share', package_name, 'launch'), 
         ['launch/processor.launch.py']),
        (os.path.join('share', package_name, 'config'), 
         ['config/params.yaml'])

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YourName',
    maintainer_email='you@example.com',
    description='Point cloud processing package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'processor = volume_cal.volume_cal_node:main',
        ],
    },
)