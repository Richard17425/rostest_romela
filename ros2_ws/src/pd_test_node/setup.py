from setuptools import setup

package_name = 'pd_test_node'

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
    maintainer='ruiqimao',
    maintainer_email='1529413416@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require=['pytest'],
    entry_points={
        'console_scripts': [
            'midlevel_planner_node = my_midlevel_node.midlevel_planner_node:main'
        ],
    },
)
