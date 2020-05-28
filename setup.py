from setuptools import setup
from rowma_ros2 import version

package_name = 'rowma_ros2'

setup(
    name=package_name,
    version=version.version,
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rowma= rowma_ros2.rowma:main',
        ],
    },
)
