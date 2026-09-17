from setuptools import setup

package_name = 'autodrive_neoracer'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         ['launch/sim_twin.launch.py', 'launch/twin_autonomy.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Koneshka Bandyopadhyay',
    maintainer_email='kb@neobotics.org',
    description='NeoRacer digital twin bridge for the AutoDRIVE Simulator',
    license='BSD-2-Clause',
    entry_points={
        'console_scripts': [
            'sim_twin_bridge = autodrive_neoracer.sim_twin_bridge:main',
            'wall_avoid_demo = autodrive_neoracer.wall_avoid_demo:main',
            'odom_tf_broadcaster = autodrive_neoracer.odom_tf_broadcaster:main',
        ],
    },
)
