from setuptools import find_packages, setup

package_name = 'terrain_analysis'

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
    maintainer='jycheon',
    maintainer_email='jjae058159@kau.kr',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'terrain_analysis_node = terrain_analysis.terrain_analysis_node:main',
            'terrain_analysis_node_limo = terrain_analysis.terrain_analysis_node_limo:main',
        ],
    },
)
