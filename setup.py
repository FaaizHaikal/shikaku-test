import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'shikaku_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    py_modules=[
        'shikaku_test/node/shikaku_test_node',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'meshes', 'teen_size'), glob('meshes/teen_size/*')),
        (os.path.join('share', package_name, 'meshes', 'kid_size'), glob('meshes/kid_size/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='faaiz',
    maintainer_email='faaizhaikal@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = shikaku_test.main:main',
            'marker_test = shikaku_test.marker_test:main'
        ],
    },
)
