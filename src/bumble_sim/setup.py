from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'bumble_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world'))),

        # Включение всех файлов внутри папки models и её подкаталогов
        ('share/' + package_name + '/models/aruco/materials/scripts', glob('models/aruco/materials/scripts/*')),
        ('share/' + package_name + '/models/aruco/materials/textures', glob('models/aruco/materials/textures/*')),
        ('share/' + package_name + '/models/aruco', ['models/aruco/model.config', 'models/aruco/model.sdf']),
        ('share/' + package_name + '/models/ground_plane',
         ['models/ground_plane/model.config', 'models/ground_plane/model.sdf']),

        # ('share/' + package_name + '/models/r1_rover/meshes', glob('models/r1_rover/meshes/*.STL')),
        # ('share/' + package_name + '/models/r1_rover', ['models/r1_rover/model.config', 'models/r1_rover/r1_rover.sdf']),

        ('share/' + package_name + '/models/sun', ['models/sun/model.config', 'models/sun/model.sdf']),

        # Py sctripts
        ('lib/' + package_name, glob(package_name + '/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vadim',
    maintainer_email='v.shtein@uniquerobotics.ru',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'offboard_sim_control = bumble_sim.offboard_sim_control:main',
            'offboard_control = bumble_sim.offboard_control:main',
            # 'visualizer = bumble_sim.visualizer:main',
            'math = bumble_sim.maths:main',
            'velocity_control = bumble_sim.velocity_control:main',
            'control = bumble_sim.control:main',
            'processes = bumble_sim.processes:main'
        ],
    },
)

