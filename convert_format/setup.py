from setuptools import find_packages, setup

package_name = 'convert_format'

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
    maintainer='apollo-22',
    maintainer_email='hayashi.hyt2001@icloud.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bayer_to_rgb_converter = convert_format.bayer_to_rgb_converter:main'
        ],
    },
)
