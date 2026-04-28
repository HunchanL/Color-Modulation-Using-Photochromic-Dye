from setuptools import find_packages, setup

package_name = 'octo_ctrl'

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
    maintainer='lhc15',
    maintainer_email='hclee94@bu.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'octoctrl = octo_ctrl.control_main_test:main',
            'camera = octo_ctrl.camera:main',
            'motor_ctrl = octo_ctrl.motor_ctrl_test:main',
            'motion_detect = octo_ctrl.camera_motion_detect:main',
        ],
    },
)
