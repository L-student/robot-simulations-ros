from setuptools import find_packages, setup

package_name = 'denso_vp6242'

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
    maintainer='glauberrleite',
    maintainer_email='glauberrleite@todo.todo',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_denso = denso_vp6242.simple_denso:main'
        ],
    },
)
