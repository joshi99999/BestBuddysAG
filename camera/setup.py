from setuptools import setup

package_name = 'camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seb',
    maintainer_email='seb@bestbuddys.de',
    description='Captures images from the camera.',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera = camera.camera:main'
        ],
    },
)
