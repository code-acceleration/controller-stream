from setuptools import setup

package_name = 'controller_stream'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[],
    install_requires=['setuptools', 'pyzmq', 'rclpy', 'sensor_msgs'],
    zip_safe=True,
    maintainer='cri-pc-2',
    maintainer_email='cri-pc-2@todo.todo',
    description='ROS2 to ZMQ bridge',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy_to_zmq = controller_stream.joy_to_zmq:main',
            'zmq_to_joy = controller_stream.zmq_to_joy:main',
        ],
    },
)
