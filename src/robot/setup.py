from setuptools import find_packages, setup

package_name = 'robot'

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
    maintainer='jamorett',
    maintainer_email='jamorett@espol.edu.ec',
    description='Nodo para controlar servos con PCA9685 en ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # aquí registramos tu nodo
            'servo_node = robot.servo_node:main',
            'sequence_node = robot.sequence_node:main'
        ],
    },
)