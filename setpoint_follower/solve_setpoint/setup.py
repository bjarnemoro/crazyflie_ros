from setuptools import find_packages, setup

#package_name = 'solve_setpoint'
package_name = 'solve_setpoint'
submodule_name = 'solve_setpoint/solvers'

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
    maintainer='bmoro',
    maintainer_email='bjarne.moro@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'solve_setpoint = solve_setpoint.solve_setpoint:main',
            'agent = solve_setpoint.agent:main',
            'manager = solve_setpoint.managers:main'
        ],
    },
)
