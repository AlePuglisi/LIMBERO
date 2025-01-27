from setuptools import find_packages, setup

package_name = 'grieel_gui'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resources', 
         ['resources/LIMBERO.png', 'resources/GripperLF.png', 'resources/WheelLF.png','resources/GripperLH.png', 'resources/WheelLH.png',
          'resources/GripperRH.png', 'resources/WheelRH.png','resources/GripperRF.png', 'resources/WheelRF.png', 'resources/Driving.png', 
          'resources/WheelCW.png', 'resources/WheelCCW.png', 'resources/WheelSTOP.png', 'resources/Standard.png']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ale',
    maintainer_email='puglisialessandro27@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui_button_publisher = grieel_gui.grieel_gui:main',
        ],
    },
)
