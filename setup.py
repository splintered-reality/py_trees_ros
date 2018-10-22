#!/usr/bin/env python

from setuptools import find_packages, setup

install_requires = [ # ] if os.environ.get('AMENT_PREFIX_PATH') else [
    # build
    'setuptools',
    # runtime

]

setup(
    name='py_trees_ros',
    version='0.5.14',  # also update package.xml and version.py
    packages=find_packages(exclude=['tests*', 'docs*', 'launch*']),
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
             'py-trees-blackboard-watcher = py_trees_ros.programs.blackboard_watcher:main',
             'py-trees-tree-watcher = py_trees_ros.programs.tree_watcher:main',
             # TODO: none of these should be in the global bin
             'py-trees-demo-exchange = py_trees_ros.demos.exchange:main',
             'py-trees-ros-tutorial-tree-one = py_trees_ros.tutorials.one:main',
         ],
     },
)
