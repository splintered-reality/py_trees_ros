#!/usr/bin/env python

from setuptools import find_packages, setup

package_name = 'py_trees_ros'

install_requires = [ # ] if os.environ.get('AMENT_PREFIX_PATH') else [
    # build
    'setuptools',
    # runtime

]

setup(
    name=package_name,
    version='0.5.14',  # also update package.xml and version.py
    packages=find_packages(exclude=['tests*', 'docs*', 'launch*']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        # global scripts (note: package specific scripts use the entry_points)
        ('bin', ['scripts/py-trees-blackboard-watcher',
                 'scripts/py-trees-tree-watcher']),
    ],
    install_requires=install_requires,
    extras_require={},
    author='Daniel Stonier, Naveed Usmani, Michal Staniaszek',
    maintainer='Daniel Stonier <d.stonier@gmail.com>',
    url='https://github.com/stonier/py_trees_ros',
    keywords=['ROS', 'behaviour-trees'],
    zip_safe=True,
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: BSD License',
        'Programming Language :: Python',
        'Topic :: Scientific/Engineering :: Artificial Intelligence',
        'Topic :: Software Development :: Libraries'
    ],
    description=(
        "ROS extensions for py-trees, a pythonic implementation of "
        "behaviour trees."
    ),
    long_description=(
        "ROS extensions for py-trees, a pythonic implementation of "
        "behaviour trees. It includes ROS specific behaviours a tree"
        "manager with ROS communication handles for debugging and"
        "visualisation, logging and various tutorials."
    ),
    license='BSD',
    # test_suite = 'nose.collector',
    # tests_require=['nose', 'pytest', 'flake8', 'yanc', 'nose-htmloutput']
    # tests_require=['pytest'],
    entry_points={
         'console_scripts': [
             # These are redirected to lib/<package_name> by setup.cfg
             'py-trees-demo-exchange = py_trees_ros.demos.exchange:main',
             'py-trees-ros-tutorial-tree-one = py_trees_ros.tutorials.one:main',
             'mock-battery = py_trees_ros.mock.battery:main',
             'mock-led-strip = py_trees_ros.mock.led_strip:main',
         ],
     },
)
