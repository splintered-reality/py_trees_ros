#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from setuptools import find_packages, setup

package_name = 'py_trees_ros'

setup(
    name=package_name,
    version='2.0.11',  # also package.xml, doc/conf.py, py_trees_ros/version.py
    packages=find_packages(
        exclude=['doc*', 'tests*', 'graveyard*', 'scripts*']
    ),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages', [
            'resources/py_trees_ros']),
    ],
    package_data={},
    install_requires=[],
    extras_require={},
    author='Daniel Stonier, Naveed Usmani, Michal Staniaszek',
    maintainer='Daniel Stonier <d.stonier@gmail.com>',
    url='https://github.com/splintered-reality/py_trees_ros',
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
    test_suite='tests',
    tests_require=[],  # using vanilla py unit tests
    entry_points={
        'console_scripts': [
            'py-trees-blackboard-watcher = py_trees_ros.programs.blackboard_watcher:main',
            # 'py-trees-echo = py_trees_ros.programs.echo:main',
            # 'py-trees-multi-talker = py_trees_ros.programs.multi_talker:main',
            'py-trees-tree-watcher = py_trees_ros.programs.tree_watcher:main',
        ],
    },
)
