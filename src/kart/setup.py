from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'kart'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')), 
    ],
    install_requires=['setuptools', 'gpiozero'],
    zip_safe=True,
    maintainer='sg',
    maintainer_email='sg@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'main.py = kart.main:main',
        	'pump_DC.py = kart.pump_DC:main',
        	'camera_servo.py = kart.camera_servo:main',
        	'steering_servo.py = kart.steering_servo:main',
        	'watergun_servo.py = kart.watergun_servo:main',
        	'wheel_DC.py = kart.wheel_DC:main',
        ],
    },
)
