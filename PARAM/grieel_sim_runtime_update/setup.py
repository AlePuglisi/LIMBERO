from setuptools import find_packages, setup

package_name = 'grieel_sim_runtime_update'

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
    maintainer='ale',
    maintainer_email='puglisi.alessandro.s5@dc.tohoku.ac.jp',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'grieel_sim_runtime_update = grieel_sim_runtime_update.grieel_sim_runtime_update:main'
        ],
    },
)
