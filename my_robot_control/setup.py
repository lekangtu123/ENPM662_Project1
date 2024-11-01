from setuptools import setup

package_name = 'my_robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'my_robot_control.proportional_controller',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='leoo',
    maintainer_email='lekangtu@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'proportional_controller = my_robot_control.proportional_controller:main'
        ],
    },
)
