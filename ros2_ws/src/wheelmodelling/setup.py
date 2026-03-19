from setuptools import find_packages, setup

package_name = 'wheelmodelling'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/wheelmodelling/config', ['wheelmodelling/config/step_input_params.json'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jacob',
    maintainer_email='jlinde24@student.aau.dk',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': ['step_input = wheelmodelling.step_input:main'
        ],
    },
)
