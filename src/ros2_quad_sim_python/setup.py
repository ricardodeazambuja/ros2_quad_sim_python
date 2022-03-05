from setuptools import setup, find_packages

package_name = 'ros2_quad_sim_python'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'scipy>=1.6', 'numpy>=1.20', 'quad_sim_python'],
    zip_safe=True,
    maintainer='Ricardo de Azambuja',
    maintainer_email='ricardo.azambuja@gmail.com',
    description='Quadcopter simulator and controller based on Python',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['quadsim = ros2_quad_sim_python.ros_quad_sim:main',
                            'quadctrl = ros2_quad_sim_python.ros_quad_ctrl:main'],
    },
)