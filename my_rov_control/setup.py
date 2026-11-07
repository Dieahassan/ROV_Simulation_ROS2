from setuptools import find_packages, setup

package_name = 'my_rov_control'

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
    maintainer='ale',
    maintainer_email='puglisialessandro27@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'my_rov_joint_states_simple=my_rov_control.my_rov_joint_states_simple:main',
            'my_rov_keybead_teleop=my_rov_control.my_rov_keybead_teleop:main',
            'my_odom2tf = my_rov_control.odom2tf:main',
            'odom2tf = my_rov_control.odom2tf_2:main',
        ],
    },
)
