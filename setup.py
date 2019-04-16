#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from setuptools import find_packages, setup

package_name = 'py_trees_ros'

extras_require = {
    'docs': ["Sphinx<2",
             "sphinx-argparse<0.3",
             "sphinx_rtd_theme<0.5",
             "sphinx-autodoc-typehints==1.6.0",
             'py_trees'
             ],
}

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(
        exclude=['doc*', 'tests*', 'graveyard*', 'scripts*']
    ),
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    package_data={},
    install_requires=[],
    extras_require=extras_require,
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
            'py-trees-tree-watcher = py_trees_ros.programs.tree_watcher:main',
            'py-trees-latched-echo = py_trees_ros.programs.latched_echo:main'
         ],
     },
)
