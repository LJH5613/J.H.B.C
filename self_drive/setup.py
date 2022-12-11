from setuptools import setup, find_packages

package_name = 'self_drive'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='pi',
    author_email='5613ljh@naver.com',
    maintainer='pi',
    maintainer_email='5613ljh@naver.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: TODO: License declaration',
        'Programming Language :: Python'
    ],
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'self_drive = {package_name}.self_drive:main',
        ],
    },
)
