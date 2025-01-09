from setuptools import setup
import os
from glob import glob

package_name = 'eced3901'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'models'), glob('models/*.urdf')),
        (os.path.join('share', package_name, 'models', 'eced3901bot_description'), glob('models/eced3901bot_description/*')),
        (os.path.join('share', package_name, 'models', 'eced3901_lab4'), glob('models/eced3901_lab4/*')),        
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'params'), glob('params/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@dal.ca',
    description='ECED3901 package for all things robot',
    license='GNU GPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = eced3901.publisher_member_function:main',
            'dt1 = eced3901.eced3901_dt1:main',
            'serialsensor = eced3901.eced3901_serialcode:main',
        ],
    },
)

