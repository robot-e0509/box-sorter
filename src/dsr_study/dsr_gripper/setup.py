from setuptools import find_packages, setup

package_name = 'dsr_gripper'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pl3',
    maintainer_email='kyung133851@pinklab.art',
    description='RH-P12-RN(A) 그리퍼 서비스 노드 (방식 A: DrlStart 매 호출)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gripper_service = dsr_gripper.gripper_service:main',
            'gripper_service_a = dsr_gripper.gripper_service_a:main',
        ],
    },
)
