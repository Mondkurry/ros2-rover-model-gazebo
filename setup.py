from setuptools import setup
import os
from glob import glob

package_name = 'rover'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']), 
        (os.path.join('share', package_name), glob('launch/*.py')),     # Add path for launch file .. MAKE SURE TO ADD COMMAS AFTER EACH LINE
        (os.path.join('share', package_name), glob('urdf/*')),          # Add path for urdf .. MAKE SURE TO ADD COMMAS AFTER EACH LINE
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mondkurry',
    maintainer_email='aryan.mondkar@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'obstacle avoider = rover.obstacle_avoidance:main',
        ],
    },
)
