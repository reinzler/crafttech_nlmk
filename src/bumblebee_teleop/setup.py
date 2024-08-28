from setuptools import find_packages, setup

package_name = 'bumblebee_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, ['bumblebee_teleop/bumblebee_keyboard_teleop.py']),
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
            'bumblebee_keyboard_teleop = bumblebee_teleop.bumblebee_keyboard_teleop:main'
        ],
    },
)
