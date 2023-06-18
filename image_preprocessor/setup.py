from setuptools import setup

package_name = 'image_preprocessor'

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
    description='Converts the images to black and white images, detects the scale, transforms the perspective and cuts out the conveyor beld area.',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'preprocessor = image_preprocessor.preprocessing:main'
        ],
    },
)
