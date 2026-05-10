from setuptools import find_packages, setup

package_name = 'rm_python'

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
    maintainer='zws',
    maintainer_email='zws@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require = ['pytest'],
    entry_points={
        'console_scripts': [
            'test_demo = rm_python.test_demo:main',
            'joint_demo = rm_python.joint_demo:main',
        ],
    },
)
