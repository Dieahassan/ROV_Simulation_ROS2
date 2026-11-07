from setuptools import find_packages, setup

package_name = 'my_rov_estimation'

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
            'my_rov_apriltag_estimation = my_rov_estimation.my_rov_apriltag:main',
            'my_rov_plot_estimation = my_rov_estimation.my_rov_plot:main',  
            'rov_orb_slam3 = my_rov_estimation.rov_orb_slam3:main',     
            'bridge_slam = my_rov_estimation.bridge_slam:main',
        ],
    },
)
