from setuptools import find_packages, setup

package_name = 'test_stand'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'gpiod>=2.0',
    ],
    zip_safe=True,
    maintainer='mw',
    maintainer_email='natec@vt.edu',
    description='Test stand package for HX711 load cell sensor',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'hx711_node = test_stand.hx711_node:main'
        ],
    },
)
