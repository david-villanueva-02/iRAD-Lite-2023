from setuptools import setup

package_name = 'tests_px4'

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
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "test = tests_px4.test:main",
            "test2 = tests_px4.test2:main",
            "test3 = tests_px4.test3:main",
            "test4 = tests_px4.test4:main",
            "mavsdk_test1 = tests_px4.mavsdk_test1:main",
            "mavsdk_test2 = tests_px4.mavsdk_test2:main",
            "mavsdk_test3 = tests_px4.mavsdk_test3:main",
            "test_keyboard = tests_px4.test_keyboard:main",
            "test_keyboard2 = tests_px4.test_keyboard2:main"
        ],
    },
)
