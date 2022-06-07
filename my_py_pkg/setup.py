from setuptools import setup

package_name = 'my_py_pkg'

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
    maintainer='lar',
    maintainer_email='lar@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "extname_py_node=my_py_pkg.FirstNode:main",
            "NewsStationEXT=my_py_pkg.robot_newsSTATION:main",
            "SmartphoneEXT=my_py_pkg.smartphone:main",
            "NumberPublisherEXT=my_py_pkg.number_publisher:main",
            "NumberCounterEXT=my_py_pkg.number_counter:main",
            "extname2_py_node=my_py_pkg.node_template:main",
            "AddEXT=my_py_pkg.adding_server:main",
            "AddingClientNoOOP_EXT=my_py_pkg.AddIntsClientNoOOP:main",
            "AddingClientOOP_EXT=my_py_pkg.AddIntsClient:main", 
            "ResetClientEXT=my_py_pkg.reset_number_counter:main"
            
        ],
    },
)
