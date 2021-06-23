from setuptools import setup
from glob import glob

package_name = 'dots_example_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch/', glob('launch/*.py')),
        ('share/' + package_name + '/launch/', glob('launch/*.xml')),
        ('share/' + package_name + '/launch/', glob('launch/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='simon2.jones@bristol.ac.uk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'explore = dots_example_controller.explore:main',
            'carry = dots_example_controller.carry:main',
            'RW_bias = dots_example_controller.RW_bias:main',
        ],
    },
)
