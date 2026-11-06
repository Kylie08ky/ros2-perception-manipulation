from setuptools import setup

package_name = 'arm_controller_pkg'

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
            'arm_commander = arm_controller_pkg.arm_commander:main',
        ],
    },
)

