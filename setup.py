from setuptools import setup

package_name = 'zumo_serial_comm'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='karl',
    maintainer_email='21990134+coderkarl@users.noreply.github.com',
    description='Serial ROS bridge for Zumo robot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "zumo_serial = zumo_serial_comm.zumo_serial_comm:main",
            "zumo_microros = zumo_serial_comm.zumo_microros:main"
        ],
    },
)
