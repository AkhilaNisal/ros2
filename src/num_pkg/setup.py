from setuptools import find_packages, setup

package_name = 'num_pkg'

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
    maintainer='akhila-wedamestrige',
    maintainer_email='akhila-wedamestrige@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'number_publisher = num_pkg.number_publisher:main',
            'number_counter = num_pkg.number_counter:main',
        ],
    },
)
