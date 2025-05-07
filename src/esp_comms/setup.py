from setuptools import find_packages, setup

package_name = 'esp_comms'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),  # You can use this to find all packages
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='athomas',
    maintainer_email='a99thomas@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rpi_esp_comms = esp_comms.rpi_esp_comms:main',
        ],
    },
)
