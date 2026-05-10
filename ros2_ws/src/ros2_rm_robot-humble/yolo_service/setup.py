from setuptools import find_packages, setup

package_name = 'yolo_service'

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
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolo_detector = yolo_service.yolo_detector:main',
            'yolo_grasp_planner = yolo_service.yolo_grasp_planner:main',
            'auto_grasp_fsm = yolo_service.auto_grasp_fsm:main',
            'test_gripper = yolo_service.test_gripper:main',
            'move_grasp = yolo_service.move_grasp:main',
            'auto_move_grasp = yolo_service.auto_move_grasp:main',
            'real_grasp_planner = yolo_service.real_grasp_planner:main',
            'yolo_grasp_seg_planner = yolo_service.yolo_grasp_seg_planner:main',
        ],
    },
)
