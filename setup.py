#!/usr/bin/env python

import os

from setuptools import find_packages, setup

package_name = 'py_trees_ros'

install_requires = [] if os.environ.get('AMENT_PREFIX_PATH') else [
    # build
    'setuptools',
    # runtime
]

extras_require = {} if os.environ.get('AMENT_PREFIX_PATH') else {
    'test': [],
    'docs': ["Sphinx", "sphinx-argparse", "sphinx_rtd_theme", "sphinx-autodoc-annotation"],
    'debs': ['stdeb', 'twine']
}

setup(
    name=package_name,
    version='0.5.14',  # also update package.xml and version.py
    packages=find_packages(exclude=['docs*']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    package_data={'py_trees_ros': ['mock/gui/*']},
    install_requires=install_requires,
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
    # test_suite='tests',
    # tests_require=['nose', 'pytest', 'flake8', 'yanc', 'nose-htmloutput']
    # tests_require=['pytest'],
    entry_points={
         'console_scripts': [
         ],
     },
)
