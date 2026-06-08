from setuptools import find_packages, setup

package_name = 'gobilda_utilities'

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
    maintainer='calpoly',
    maintainer_email='cdiazalv@calpoly.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'draw_square_fb = gobilda_utilities.draw_square_fb:main',
            'teleop_gobilda = gobilda_utilities.teleop_twist_keyboard:main',
            'bump_and_go = gobilda_utilities.control_node:main',
        ],
    },
)
