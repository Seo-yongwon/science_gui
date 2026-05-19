from setuptools import setup
import os
from glob import glob

package_name = 'science_gui'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch*.py'))),
        (os.path.join('share', package_name, 'systemd'),
            glob(os.path.join('systemd', '*'))),
        (os.path.join('share', package_name, 'calibration'),
            glob(os.path.join('calibration', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Rover science mission: 4-camera merge, GStreamer UDP stream, sensor bridges, and basestation GUI.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cam_driver       = science_gui.cam_driver:main',
            'gst_sender       = science_gui.gst_sender:main',
            'gst_receiver     = science_gui.gst_receiver:main',
            'mission_gui      = science_gui.mission_gui:main',
            'scilab_bridge    = science_gui.scilab_bridge:main',
            'npk_bridge       = science_gui.npk_bridge:main',
            'mission_panorama = science_gui.mission_panorama:main',
            'panorama         = science_gui.mission_panorama:main',
            'rover_launch_trigger = science_gui.rover_launch_trigger:main',
        ],
    },
)
