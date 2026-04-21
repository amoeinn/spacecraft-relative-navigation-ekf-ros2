from setuptools import find_packages, setup

package_name = 'space_sim_py'

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
    maintainer='amoein',
    maintainer_email='moein.bme@gmail.com',
    description='Space simulation nodes (truth + sensor)',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'truth_sim_node = space_sim_py.truth_sim_node:main',
            'sensor_sim_node = space_sim_py.sensor_sim_node:main',
        ],
    },
)
