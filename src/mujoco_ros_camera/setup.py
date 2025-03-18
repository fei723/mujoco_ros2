from setuptools import find_packages, setup


from glob import glob #这里
import os #这里



package_name = 'mujoco_ros_camera'
def get_all_files_in_dir(directory):
    all_files = []
    for root, dirs, files in os.walk(directory):
        for file in files:
            all_files.append(os.path.join(root, file))
    return all_files
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'xml/MI_arm'), glob('xml/MI_arm/*.xml')),
        (os.path.join('share', package_name, 'xml/MI_arm/mesh'), glob('xml/MI_arm/mesh/*.obj')),
        (os.path.join('share', package_name, 'xml/MI_arm/mesh/DH_claw_v2'), glob('xml/MI_arm/mesh/DH_claw_v2/*')),
        (os.path.join('share', package_name, 'xml/MI_arm/mesh/RM75_6F'), glob('xml/MI_arm/mesh/RM75_6F/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'srv'), glob('srv/*.srv')),
    
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shifei',
    maintainer_email='shifei3@xiaomi.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mujoco_camera_node = mujoco_ros_camera.mujoco_camera_node:main',
            'camera_show_node = mujoco_ros_camera.camera_show_node:main',
            'test_camera = mujoco_ros_camera.test_camera:main',
            'test_show_node = mujoco_ros_camera.test_show_node:main',
            'image_processing_action_server = mujoco_ros_camera.image_processing_action_server:main',
            'test_IK_serv = mujoco_ros_camera.test_IK_serv:main',
            'test_PID_control = mujoco_ros_camera.test_PID_control:main',
        ],
    },
)
