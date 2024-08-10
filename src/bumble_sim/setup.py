# from setuptools import find_packages, setup
# import os
# from glob import glob
#
# package_name = 'bumble_sim'
#
# setup(
#     name=package_name,
#     version='0.0.0',
#     packages=find_packages(exclude=['test']),
#     data_files=[
#         ('share/ament_index/resource_index/packages',
#             ['resource/' + package_name]),
#         ('share/' + package_name, ['package.xml']),
#         (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
#         (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world'))),
#
#         # Устанавливаем SDF модели
#         ('share/' + package_name + '/models/r1_rover/meshes',
#          glob('models/r1_rover/meshes/*.STL')),
#         ('share/' + package_name + '/models/r1_rover',
#          ['models/r1_rover/model.config', 'models/r1_rover/r1_rover.sdf']),
#         ('share/' + package_name + '/launch',
#          glob('launch/*.py')),
#     ],
#     install_requires=['setuptools'],
#     zip_safe=True,
#     maintainer='vadim',
#     maintainer_email='v.shtein@uniquerobotics.ru',
#     description='TODO: Package description',
#     license='TODO: License declaration',
#     tests_require=['pytest'],
#     entry_points={
#         'console_scripts': [
#         ],
#     },
# )

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

        # Устанавливаем SDF модели
        ('share/' + package_name + '/models/r1_rover/meshes',
         glob('models/r1_rover/meshes/*.STL')),
        ('share/' + package_name + '/models/r1_rover',
         ['models/r1_rover/model.config', 'models/r1_rover/r1_rover.sdf',
          'models/r1_rover/r1_rover.sdf.jinja', 'models/r1_rover/r1_rover.sdf.last_generated']),
        ('share/' + package_name + '/models',
         glob('models/*.sdf')),
        ('share/' + package_name + '/models/r1_rover',
         glob('models/r1_rover/*.sdf')),
        ('share/' + package_name + '/launch',
         glob('launch/*.launch.py')),

        # Offboard control
        ('lib/' + package_name, [package_name + '/offboard_control/' + 'offboard_sim_control.py']),
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
            'offboard_control = offboard_sim_control:main',
        ],
    },
)

