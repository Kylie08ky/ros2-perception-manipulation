from setuptools import setup

package_name = 'sensorfusion_arm_controller_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Control arm joints with joint trajectory message',
    license='MIT',
    entry_points={
        'console_scripts': [
            'arm_commander = sensorfusion_arm_controller_pkg.sensorfusion_arm_commander:main',
        ],
    },
)

