from setuptools import setup

package_name = 'erc_gui'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your@email.com',
    description='ERC GUI for camera and control',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gui_node = erc_gui.gui_node:main'
        ],
    },
)
