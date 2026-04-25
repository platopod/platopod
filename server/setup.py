from setuptools import setup, find_packages

package_name = 'plato_pod'

setup(
    name=package_name,
    version='0.1.0',
    package_dir={'': 'src'},
    packages=find_packages(where='src'),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/vision.launch.py',
            'launch/arena.launch.py',
            'launch/registry.launch.py',
            'launch/gateway.launch.py',
            'launch/virtual_sim.launch.py',
            'launch/gazebo.launch.py',
            'launch/simulation.launch.py',
            'launch/sensor_engine.launch.py',
            'launch/cot_bridge.launch.py',
            'launch/atak_test.launch.py',
            'launch/terrain.launch.py',
            'launch/replay.launch.py',
            'launch/classroom.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/apriltag_settings.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Plato Pod Team',
    maintainer_email='dev@platopod.io',
    description='Plato Pod AR tactical simulation platform server',
    license='MIT',
    entry_points={
        'console_scripts': [
            'vision_node = plato_pod.vision_node:main',
            'arena_model_node = plato_pod.arena_model_node:main',
            'registry_node = plato_pod.registry_node:main',
            'robot_bridge_node = plato_pod.robot_bridge_node:main',
            'api_gateway_node = plato_pod.api_gateway_node:main',
            'virtual_sim_node = plato_pod.virtual_sim_node:main',
            'sensor_engine_node = plato_pod.sensor_engine_node:main',
            'cot_bridge_node = plato_pod.cot_bridge_node:main',
            'gazebo_bridge_node = plato_pod.gazebo_bridge_node:main',
            'replay_node = plato_pod.replay_node:main',
            'camera_calibrate = plato_pod.calibration:main',
        ],
    },
)
