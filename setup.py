from setuptools import setup, find_packages

package_name = 'ft_fsd'

setup(
    name=package_name,
    version='0.0.0',
    packages=['ft_fsd', 'fsd_path_planning/fsd_path_planning'],
    package_dir={
        'fsd_path_planning' : 'fsd_path_planning/fsd_path_planning'
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='naza',
    maintainer_email='uzoukwuc@tcd.ie',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ft_fsd = ft_fsd.ft_fsd:main'
        ],
    },
)
