from glob import glob
from setuptools import setup

setup(
    name='flyco_foxglove', version='0.1.0', packages=['flyco_foxglove'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/flyco_foxglove']),
        ('share/flyco_foxglove', ['package.xml']),
        ('share/flyco_foxglove/launch', glob('launch/*.launch.py')),
        ('share/flyco_foxglove/config', glob('config/*.json')),
    ],
    install_requires=['setuptools', 'PyYAML'], zip_safe=True,
    maintainer='YuanLinLiu', maintainer_email='2576640645@qq.com',
    description='Independent RViz layout export and Foxglove visualization.',
    license='PolyForm-Noncommercial-1.0.0',
    entry_points={'console_scripts': [
        'rviz_to_foxglove = flyco_foxglove.node:main',
        'export_foxglove_layout = flyco_foxglove.converter:main',
    ]},
)
