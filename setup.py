from setuptools import find_packages, setup

package_name = 'kinova_gen3_lite_teleoperation'

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
    maintainer='choiyj',
    maintainer_email='cyj21c6352@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2_bridge = kinova_gen3_lite_teleoperation.kinova_control:main',
            'master_dxl_bridge = kinova_gen3_lite_teleoperation.master_control:main',
        ],
    },
)
