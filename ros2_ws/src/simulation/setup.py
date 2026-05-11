from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'),
                    glob('launch/*.py')),

	(os.path.join('share', package_name, 'worlds'),
        	glob('worlds/*.world')),
        
	(os.path.join('share', package_name, 'models'),
                [f for f in glob('models/**/*', recursive=True) if os.path.isfile(f)]
                ),
                (os.path.join('share', package_name, 'urdf'),
                    glob('urdf/*')),
    
    (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
