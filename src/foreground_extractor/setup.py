from setuptools import setup
import os

package_name = 'foreground_extractor'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
         ['launch/extractor.launch.py']),
    ],
    install_requires=['setuptools', 'open3d', 'numpy'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Foreground extractor using voxel hashing and height check',
    license='MIT',
    entry_points={
        'console_scripts': [
            'foreground_extractor = foreground_extractor.foreground_extractor:main',
        ],
    },
)
