from setuptools import find_packages, setup

package_name = 'basic_package'

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
    maintainer='senku',
    maintainer_email='senku@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "basic_node1 = basic_package.basic_node1:main",
            "basic_node2 = basic_package.basic_node2:main",
            "basic_node3 = basic_package.basic_node3:main"
        ],
    },
)
