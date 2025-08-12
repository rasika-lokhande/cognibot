from setuptools import find_packages, setup

package_name = 'common_coding'

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
    maintainer='rasika-lokhande',
    maintainer_email='rasika246@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ 
            'lidar_processor=common_coding.lidar_processor:main',
            'action_executor=common_coding.action_executor:main',
        ],
    },
)
