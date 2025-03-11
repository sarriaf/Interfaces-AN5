from setuptools import find_packages, setup

package_name = 'code'

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
    maintainer='angel',
    maintainer_email='angel@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_publisher = code.robot_publisher:main',
            'robot_publicador = code.robot_publicador:main',
            'Publicador_tcp = code.Publicador_tcp:main',
            'command_client = code.command_client:main',
            'Publicador_Subscritor = code.Publicador_Subscritor:main',
            'robot_controller = code.robot_controller:main',
            'command_subscriber = code.command_subscriber:main',
            'robot_teleop = code.robot_teleop:main',
            'command_server = code.command_server:main', 
            'troubleshooting = code.troubleshooting:main',
            'Inversa_suscriber = code.Inversa_suscriber:main',
            'command_publisher = code.command_publisher:main',
            'mgd = code.mgd:main',
            'mgd1 = code.mgd1:main',
            'publisher_subscriber = code.publisher_subscriber:main',
            'MoveL = code.MoveL:main',
            


        ],
    },
)
