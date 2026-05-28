from setuptools import setup

package_name = 'rovio_qos_relay'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hann',
    maintainer_email='hann@example.com',
    description='QoS relay for RealSense D435i to ROVIO',
    license='MIT',
    entry_points={
        'console_scripts': [
            'd435i_to_rovio_relay = rovio_qos_relay.d435i_to_rovio_relay:main',
        ],
    },
)
