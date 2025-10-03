from setuptools import setup, find_packages

package_name = 'py_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nautilus Team',
    maintainer_email='nautilus@example.com',
    description='Polaris UUV Python ROS2 nodes',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dive_test = py_pkg.dive_test:main',
            'depth_control_node = py_pkg.bladder_control.depth_control_node:main',
        ],
    },
)
