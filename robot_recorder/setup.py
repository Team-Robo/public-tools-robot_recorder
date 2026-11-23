from setuptools import setup

package_name = 'robot_recorder'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nanyang Technological University Team Robo',
    maintainer_email='teamrobontu@gmail.com',
    description='Record and replay robot-view bags.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'record = robot_recorder.record:main',
            'play = robot_recorder.play:main',
            'export_video = robot_recorder.export_video:main',
            'image_compressor = robot_recorder.image_compressor:main',
        ],
    },
)
