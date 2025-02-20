from setuptools import find_packages, setup

package_name = 'mapping'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/cartographer_velodyne.launch.py']),
    ],
    install_requires=['setuptools',
    		      'cartographer_ros',
    		      'rviz2',
    		      'sensor_msgs',
    		      'geometry_msgs',
    		      'tf2_ros',],
    zip_safe=True,
    maintainer='tminh',
    maintainer_email='trantuanminh14022003@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom = data.odom:main',
            'odom2 = data.odom2:main',
        ],
    },
)
