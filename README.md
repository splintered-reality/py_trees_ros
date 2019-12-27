# Py Trees for ROS

Behaviours, trees and utilities that extend py_trees for use
with ROS.

## Documentation

* Basics & Core Module API: [![docs][py-trees-docs-eloquent-image]][py-trees-docs-2.0.x] [![docs][py-trees-docs-dashing-image]][py-trees-docs-1.3.x]
* ROS2 Module API: [![docs][py-trees-ros-docs-eloquent-image]][py-trees-ros-docs-2.0.x] [![docs][py-trees-ros-docs-dashing-image]][py-trees-ros-docs-1.2.x]
* ROS2 Tutorials: [![docs][py-trees-ros-tutorials-docs-eloquent-image]][py-trees-ros-tutorials-docs-2.0.x] [![docs][py-trees-ros-tutorials-docs-dashing-image]][py-trees-ros-tutorials-docs-1.0.x]

For older versions of the documentation, refer to the links in the matrix below.

## PyTrees-ROS Ecosystem


| ROS2 | [Eloquent][eloquent-build-farm] | [Dashing][dashing-build-farm] |  ROS1 | [Melodic][melodic-build-farm] | [Kinetic][kinetic-build-farm] |
|:---:|:---:|:---:|:---:|:---:|:---:|
| [py_trees][py-trees-ros-index] | [![2.0.x][2.0.x-sources-image]][py-trees-sources-2.0.x]<br/>[![Build Status][py-trees-build-status-eloquent-image]][py-trees-build-status-eloquent]<br/>[![2.0.x-Docs][2.0.x-rtd-image]][py-trees-docs-2.0.x] | [![1.3.x][1.3.x-sources-image]][py-trees-sources-1.3.x]<br/>[![Build Status][py-trees-build-status-dashing-image]][py-trees-build-status-dashing]<br/>[![1.3.x-Docs][1.3.x-rtd-image]][py-trees-docs-1.3.x] | [py_trees][py-trees-wiki] | [![0.6.x][0.6.x-sources-image]][py-trees-sources-0.6.x]<br/>[![Build Status][py-trees-build-status-melodic-image]][py-trees-build-status-melodic]<br/>[![Docs Status][py-trees-docs-melodic-image]][py-trees-docs-melodic] | [![0.5.x][0.5.x-sources-image]][py-trees-sources-0.5.x]<br/>[![Build Status][py-trees-build-status-kinetic-image]][py-trees-build-status-kinetic]<br/>[![Docs Status][py-trees-docs-kinetic-image]][py-trees-docs-kinetic] |
| [py_trees_ros_interfaces][py-trees-ros-interfaces-ros-index] | [![2.0.x][2.0.x-sources-image]][py-trees-ros-interfaces-sources-2.0.x]<br/>[![Build Status][py-trees-ros-interfaces-build-status-eloquent-image]][py-trees-ros-interfaces-build-status-eloquent]<br/>![1.2.x-Docs][not-available-docs-image] | [![1.2.x][1.2.x-sources-image]][py-trees-ros-interfaces-sources-1.2.x]<br/>[![Build Status][py-trees-ros-interfaces-build-status-dashing-image]][py-trees-ros-interfaces-build-status-dashing]<br/>![1.2.x-Docs][not-available-docs-image] | [py_trees_msgs][py-trees-msgs-wiki] | [![0.3.x][0.3.x-sources-image]][py-trees-msgs-sources-melodic]<br/>[![Build Status][py-trees-msgs-build-status-melodic-image]][py-trees-msgs-build-status-melodic]<br/>![0.3.x-Docs][not-available-docs-image] | [![0.3.x][0.3.x-sources-image]][py-trees-msgs-sources-kinetic]<br/>[![Build Status][py-trees-msgs-build-status-kinetic-image]][py-trees-msgs-build-status-kinetic]<br/>![0.3.x-Docs][not-available-docs-image] |
| [py_trees_ros][py-trees-ros-ros-index] | [![2.0.x][2.0.x-sources-image]][py-trees-ros-sources-2.0.x]<br/>[![Build Status][py-trees-ros-build-status-eloquent-image]][py-trees-ros-build-status-eloquent]<br/>[![2.0.x-Docs][2.0.x-rtd-image]][py-trees-ros-docs-2.0.x] | [![1.2.x][1.2.x-sources-image]][py-trees-ros-sources-1.2.x]<br/>[![Build Status][py-trees-ros-build-status-dashing-image]][py-trees-ros-build-status-dashing]<br/>[![1.2.x-Docs][1.2.x-rtd-image]][py-trees-ros-docs-1.2.x] | [py_trees_ros][py-trees-ros-wiki] | [![0.5.x][0.5.x-sources-image]][py-trees-ros-sources-0.5.x]<br/>[![Build Status][py-trees-ros-build-status-melodic-image]][py-trees-ros-build-status-melodic]<br/>[![Docs Status][py-trees-ros-docs-melodic-image]][py-trees-ros-docs-melodic] | [![0.5.x][0.5.x-sources-image]][py-trees-ros-sources-0.5.x]<br/>[![Build Status][py-trees-ros-build-status-kinetic-image]][py-trees-ros-build-status-kinetic]<br/>[![Docs Status][py-trees-ros-docs-kinetic-image]][py-trees-ros-docs-kinetic] |
| [py_trees_ros_tutorials][py-trees-ros-tutorials-ros-index] | [![2.0.x][2.0.x-sources-image]][py-trees-ros-tutorials-sources-2.0.x]<br/>[![Build Status][py-trees-ros-tutorials-build-status-eloquent-image]][py-trees-ros-tutorials-build-status-eloquent]<br/>[![2.0.x-Docs][2.0.x-rtd-image]][py-trees-ros-tutorials-docs-2.0.x] | [![1.0.x][1.0.x-sources-image]][py-trees-ros-tutorials-sources-1.0.x]<br/>[![Build Status][py-trees-ros-tutorials-build-status-dashing-image]][py-trees-ros-tutorials-build-status-dashing]<br/>[![1.0.x-Docs][1.0.x-rtd-image]][py-trees-ros-tutorials-docs-1.0.x] | - | - | - |
| [py_trees_js][py-trees-js-ros-index] | [![0.6.x][0.6.x-sources-image]][py-trees-js-sources-0.6.x]<br/>[![Build Status][py-trees-js-build-status-eloquent-image]][py-trees-js-build-status-eloquent]<br/> [![0.6.x-Docs][readme-docs-image]][py-trees-js-docs-0.6.x] | [![0.5.x][0.5.x-sources-image]][py-trees-js-sources-0.5.x]<br/>[![Build Status][py-trees-js-build-status-dashing-image]][py-trees-js-build-status-dashing]<br/> [![0.5.x-Docs][readme-docs-image]][py-trees-js-docs-0.5.x] | - | - | - |
| [py_trees_ros_viewer][py-trees-ros-viewer-ros-index] | [![0.2.x][0.2.x-sources-image]][py-trees-ros-viewer-sources-0.2.x]<br/>[![Build Status][py-trees-ros-viewer-build-status-eloquent-image]][py-trees-ros-viewer-build-status-eloquent]<br/> [![0.2.x-Docs][readme-docs-image]][py-trees-ros-viewer-docs-0.2.x] | [![0.1.x][0.1.x-sources-image]][py-trees-ros-viewer-sources-0.1.x]<br/>[![Build Status][py-trees-ros-viewer-build-status-dashing-image]][py-trees-ros-viewer-build-status-dashing]<br/> [![0.1.x-Docs][readme-docs-image]][py-trees-ros-viewer-docs-0.1.x] | [rqt_py_trees][rqt-py-trees-wiki] | [![0.3.x][0.3.x-sources-image]][rqt-py-trees-sources-melodic]<br/>[![Build Status][rqt-py-trees-build-status-melodic-image]][rqt-py-trees-build-status-melodic] | [![0.3.x][0.3.x-sources-image]][rqt-py-trees-sources-kinetic]<br/>[![Build Status][rqt-py-trees-build-status-kinetic-image]][rqt-py-trees-build-status-kinetic] |

[devel-sources-image]: http://img.shields.io/badge/sources-devel-blue.svg?style=plastic
[2.0.x-sources-image]: http://img.shields.io/badge/sources-2.0.x-blue.svg?style=plastic
[1.3.x-sources-image]: http://img.shields.io/badge/sources-1.3.x-blue.svg?style=plastic
[1.2.x-sources-image]: http://img.shields.io/badge/sources-1.2.x-blue.svg?style=plastic
[1.1.x-sources-image]: http://img.shields.io/badge/sources-1.1.x-blue.svg?style=plastic
[1.0.x-sources-image]: http://img.shields.io/badge/sources-1.0.x-blue.svg?style=plastic
[0.6.x-sources-image]: http://img.shields.io/badge/sources-0.6.x-blue.svg?style=plastic
[0.5.x-sources-image]: http://img.shields.io/badge/sources-0.5.x-blue.svg?style=plastic
[0.4.x-sources-image]: http://img.shields.io/badge/sources-0.4.x-blue.svg?style=plastic
[0.3.x-sources-image]: http://img.shields.io/badge/sources-0.3.x-blue.svg?style=plastic
[0.2.x-sources-image]: http://img.shields.io/badge/sources-0.2.x-blue.svg?style=plastic
[0.1.x-sources-image]: http://img.shields.io/badge/sources-0.1.x-blue.svg?style=plastic

[devel-rtd-image]: https://readthedocs.org/projects/py-trees/badge/?version=devel&style=plastic
[2.0.x-rtd-image]: https://readthedocs.org/projects/py-trees/badge/?version=release-2.0.x&style=plastic
[1.3.x-rtd-image]: https://readthedocs.org/projects/py-trees/badge/?version=release-1.3.x&style=plastic
[1.2.x-rtd-image]: https://readthedocs.org/projects/py-trees/badge/?version=release-1.2.x&style=plastic
[1.1.x-rtd-image]: https://readthedocs.org/projects/py-trees/badge/?version=release-1.0.x&style=plastic
[1.0.x-rtd-image]: https://readthedocs.org/projects/py-trees/badge/?version=release-1.0.x&style=plastic
[0.6.x-rtd-image]: https://readthedocs.org/projects/py-trees/badge/?version=release-0.6.x&style=plastic
[0.5.x-rtd-image]: https://readthedocs.org/projects/py-trees/badge/?version=release-0.5.x&style=plastic

[devel-docs-image]: http://img.shields.io/badge/docs-devel-brightgreen.svg?style=plastic
[1.3.x-docs-image]: http://img.shields.io/badge/docs-1.3.x-brightgreen.svg?style=plastic
[1.2.x-docs-image]: http://img.shields.io/badge/docs-1.2.x-brightgreen.svg?style=plastic
[0.6.x-docs-image]: http://img.shields.io/badge/docs-0.6.x-brightgreen.svg?style=plastic
[0.5.x-docs-image]: http://img.shields.io/badge/docs-0.5.x-brightgreen.svg?style=plastic
[0.3.x-docs-image]: http://img.shields.io/badge/docs-0.3.x-brightgreen.svg?style=plastic
[not-available-docs-image]: http://img.shields.io/badge/docs-n/a-yellow.svg?style=plastic
[readme-docs-image]: http://img.shields.io/badge/docs-README-brightgreen.svg?style=plastic

[eloquent-build-farm]: http://repo.ros2.org/status_page/ros_eloquent_default.html?q=py_trees
[dashing-build-farm]: http://repo.ros2.org/status_page/ros_dashing_default.html?q=py_trees
[melodic-build-farm]: http://repositories.ros.org/status_page/ros_melodic_default.html?q=py_trees
[kinetic-build-farm]: http://repositories.ros.org/status_page/ros_kinetic_default.html?q=py_trees

[py-trees-build-status-eloquent]: http://build.ros2.org/job/Ebin_uB64__py_trees__ubuntu_bionic_amd64__binary/
[py-trees-build-status-eloquent-image]: http://build.ros2.org/job/Ebin_uB64__py_trees__ubuntu_bionic_amd64__binary/badge/icon?style=plastic
[py-trees-build-status-dashing]: http://build.ros2.org/job/Dbin_uB64__py_trees__ubuntu_bionic_amd64__binary/
[py-trees-build-status-dashing-image]: http://build.ros2.org/job/Dbin_uB64__py_trees__ubuntu_bionic_amd64__binary/badge/icon?style=plastic
[py-trees-build-status-kinetic]: http://build.ros.org/job/Kbin_uX64__py_trees__ubuntu_xenial_amd64__binary
[py-trees-build-status-kinetic-image]: http://build.ros.org/job/Kbin_uX64__py_trees__ubuntu_xenial_amd64__binary/badge/icon?style=plastic
[py-trees-build-status-melodic]: http://build.ros.org/job/Mbin_uB64__py_trees__ubuntu_bionic_amd64__binary
[py-trees-build-status-melodic-image]: http://build.ros.org/job/Mbin_uB64__py_trees__ubuntu_bionic_amd64__binary/badge/icon?style=plastic
[py-trees-docs-devel]: http://py-trees.readthedocs.io/
[py-trees-docs-2.0.x]: http://py-trees.readthedocs.io/en/release-2.0.x/
[py-trees-docs-1.3.x]: http://py-trees.readthedocs.io/en/release-1.3.x/
[py-trees-docs-0.6.x]: http://py-trees.readthedocs.io/en/release-0.6.x/
[py-trees-docs-0.5.x]: http://docs.ros.org/kinetic/api/py_trees/html/
[py-trees-docs-kinetic]: http://docs.ros.org/kinetic/api/py_trees/html/
[py-trees-docs-eloquent-image]: http://img.shields.io/badge/py_trees-eloquent-brightgreen.svg?style=plastic
[py-trees-docs-dashing-image]: http://img.shields.io/badge/py_trees-dashing-brightgreen.svg?style=plastic
[py-trees-docs-kinetic-image]: https://img.shields.io/jenkins/s/http/build.ros.org/job/Kdoc__py_trees__ubuntu_xenial_amd64.svg?label=docs&style=plastic
[py-trees-docs-melodic]: http://docs.ros.org/melodic/api/py_trees/html/
[py-trees-docs-melodic-image]: https://img.shields.io/jenkins/s/http/build.ros.org/job/Mdoc__py_trees__ubuntu_bionic_amd64.svg?label=docs&style=plastic
[py-trees-ros-index]: https://index.ros.org/p/py_trees/github-splintered-reality-py_trees
[py-trees-sources-devel]: https://github.com/splintered-reality/py_trees/tree/devel
[py-trees-sources-2.0.x]: https://github.com/splintered-reality/py_trees/tree/release/2.0.x
[py-trees-sources-1.3.x]: https://github.com/splintered-reality/py_trees/tree/release/1.3.x
[py-trees-sources-0.6.x]: https://github.com/splintered-reality/py_trees/tree/release/0.6.x
[py-trees-sources-0.5.x]: https://github.com/splintered-reality/py_trees/tree/release/0.5.x
[py-trees-wiki]: http://wiki.ros.org/py_trees

[py-trees-ros-interfaces-build-status-eloquent]: http://build.ros2.org/job/Ebin_uB64__py_trees_ros_interfaces__ubuntu_bionic_amd64__binary/
[py-trees-ros-interfaces-build-status-eloquent-image]: http://build.ros2.org/job/Ebin_uB64__py_trees_ros_interfaces__ubuntu_bionic_amd64__binary/badge/icon?style=plastic
[py-trees-ros-interfaces-build-status-dashing]: http://build.ros2.org/job/Dbin_uB64__py_trees_ros_interfaces__ubuntu_bionic_amd64__binary/
[py-trees-ros-interfaces-build-status-dashing-image]: http://build.ros2.org/job/Dbin_uB64__py_trees_ros_interfaces__ubuntu_bionic_amd64__binary/badge/icon?style=plastic
[py-trees-ros-interfaces-ros-index]: https://index.ros.org/p/py_trees_ros_interfaces/github-splintered-reality-py_trees_ros_interfaces
[py-trees-ros-interfaces-sources-2.0.x]: https://github.com/splintered-reality/py_trees_ros_interfaces/tree/release/2.0.x
[py-trees-ros-interfaces-sources-1.2.x]: https://github.com/splintered-reality/py_trees_ros_interfaces/tree/release/1.2.x
[py-trees-ros-interfaces-sources-1.1.x]: https://github.com/splintered-reality/py_trees_ros_interfaces/tree/release/1.1.x

[py-trees-ros-build-status-eloquent]: http://build.ros2.org/job/Ebin_uB64__py_trees_ros__ubuntu_bionic_amd64__binary/
[py-trees-ros-build-status-eloquent-image]: http://build.ros2.org/job/Ebin_uB64__py_trees_ros__ubuntu_bionic_amd64__binary/badge/icon?style=plastic
[py-trees-ros-build-status-dashing]: http://build.ros2.org/job/Dbin_uB64__py_trees_ros__ubuntu_bionic_amd64__binary/
[py-trees-ros-build-status-dashing-image]: http://build.ros2.org/job/Dbin_uB64__py_trees_ros__ubuntu_bionic_amd64__binary/badge/icon?style=plastic
[py-trees-ros-build-status-kinetic]: http://build.ros.org/job/Kbin_uX64__py_trees_ros__ubuntu_xenial_amd64__binary
[py-trees-ros-build-status-kinetic-image]: http://build.ros.org/job/Kbin_uX64__py_trees_ros__ubuntu_xenial_amd64__binary/badge/icon?style=plastic
[py-trees-ros-build-status-melodic]: http://build.ros.org/job/Mbin_uB64__py_trees_ros__ubuntu_bionic_amd64__binary
[py-trees-ros-build-status-melodic-image]: http://build.ros.org/job/Mbin_uB64__py_trees_ros__ubuntu_bionic_amd64__binary/badge/icon?style=plastic
[py-trees-ros-docs-2.0.x]: http://py-trees-ros.readthedocs.io/en/release-2.0.x/
[py-trees-ros-docs-1.3.x]: http://py-trees-ros.readthedocs.io/en/release-1.3.x/
[py-trees-ros-docs-1.2.x]: http://py-trees-ros.readthedocs.io/en/release-1.2.x/
[py-trees-ros-docs-eloquent-image]: http://img.shields.io/badge/py_trees_ros-eloquent-brightgreen.svg?style=plastic
[py-trees-ros-docs-dashing-image]: http://img.shields.io/badge/py_trees_ros-dashing-brightgreen.svg?style=plastic
[py-trees-ros-docs-kinetic]: http://docs.ros.org/kinetic/api/py_trees_ros/html/
[py-trees-ros-docs-kinetic-image]: https://img.shields.io/jenkins/s/http/build.ros.org/job/Kdoc__py_trees_ros__ubuntu_xenial_amd64.svg?label=docs&style=plastic
[py-trees-ros-docs-melodic]: http://docs.ros.org/melodic/api/py_trees_ros/html/
[py-trees-ros-docs-melodic-image]: https://img.shields.io/jenkins/s/http/build.ros.org/job/Mdoc__py_trees_ros__ubuntu_bionic_amd64.svg?label=docs&style=plastic
[py-trees-ros-ros-index]: https://index.ros.org/p/py_trees_ros/github-splintered-reality-py_trees_ros
[py-trees-ros-sources-2.0.x]: https://github.com/splintered-reality/py_trees_ros/tree/release/2.0.x
[py-trees-ros-sources-1.3.x]: https://github.com/splintered-reality/py_trees_ros/tree/release/1.3.x
[py-trees-ros-sources-1.2.x]: https://github.com/splintered-reality/py_trees_ros/tree/release/1.2.x
[py-trees-ros-sources-0.5.x]: https://github.com/splintered-reality/py_trees_ros/tree/release/0.5.x
[py-trees-ros-wiki]: http://wiki.ros.org/py_trees_ros


[py-trees-ros-tutorials-build-status-eloquent]: http://build.ros2.org/job/Ebin_uB64__py_trees_ros_tutorials__ubuntu_bionic_amd64__binary/
[py-trees-ros-tutorials-build-status-eloquent-image]: http://build.ros2.org/job/Ebin_uB64__py_trees_ros_tutorials__ubuntu_bionic_amd64__binary/badge/icon?style=plastic
[py-trees-ros-tutorials-build-status-dashing]: http://build.ros2.org/job/Dbin_uB64__py_trees_ros_tutorials__ubuntu_bionic_amd64__binary/
[py-trees-ros-tutorials-build-status-dashing-image]: http://build.ros2.org/job/Dbin_uB64__py_trees_ros_tutorials__ubuntu_bionic_amd64__binary/badge/icon?style=plastic
[py-trees-ros-tutorials-docs-2.0.x]: http://py-trees-ros-tutorials.readthedocs.io/en/release-2.0.x/
[py-trees-ros-tutorials-docs-1.0.x]: http://py-trees-ros-tutorials.readthedocs.io/en/release-1.0.x/
[py-trees-ros-tutorials-sources-2.0.x]: https://github.com/splintered-reality/py_trees_ros_tutorials/tree/release/2.0.x
[py-trees-ros-tutorials-sources-1.0.x]: https://github.com/splintered-reality/py_trees_ros_tutorials/tree/release/1.0.x
[py-trees-ros-tutorials-ros-index]: https://index.ros.org/p/py_trees_ros_tutorials/github-splintered-reality-py_trees_ros_tutorials
[py-trees-ros-tutorials-docs-eloquent-image]: http://img.shields.io/badge/py_trees_ros_tutorials-eloquent-brightgreen.svg?style=plastic
[py-trees-ros-tutorials-docs-dashing-image]: http://img.shields.io/badge/py_trees_ros_tutorials-dashing-brightgreen.svg?style=plastic

[py-trees-js-build-status-eloquent]: http://build.ros2.org/job/Ebin_uB64__py_trees_js__ubuntu_bionic_amd64__binary/
[py-trees-js-build-status-eloquent-image]: http://build.ros2.org/job/Ebin_uB64__py_trees_js__ubuntu_bionic_amd64__binary/badge/icon?style=plastic
[py-trees-js-build-status-dashing]: http://build.ros2.org/job/Dbin_uB64__py_trees_js__ubuntu_bionic_amd64__binary/
[py-trees-js-build-status-dashing-image]: http://build.ros2.org/job/Dbin_uB64__py_trees_js__ubuntu_bionic_amd64__binary/badge/icon?style=plastic
[py-trees-js-docs-0.6.x]: https://github.com/splintered-reality/py_trees_js/blob/release/0.6.x/README.md
[py-trees-js-sources-0.6.x]: https://github.com/splintered-reality/py_trees_js/tree/release/0.6.x
[py-trees-js-docs-0.5.x]: https://github.com/splintered-reality/py_trees_js/blob/release/0.5.x/README.md
[py-trees-js-sources-0.5.x]: https://github.com/splintered-reality/py_trees_js/tree/release/0.5.x
[py-trees-js-docs-0.4.x]: https://github.com/splintered-reality/py_trees_js/blob/release/0.4.x/README.md
[py-trees-js-sources-0.4.x]: https://github.com/splintered-reality/py_trees_js/tree/release/0.4.x
[py-trees-js-ros-index]: https://index.ros.org/p/py_trees_js/github-splintered-reality-py_trees_js

[py-trees-ros-viewer-build-status-eloquent]: http://build.ros2.org/job/Ebin_uB64__py_trees_ros_viewer__ubuntu_bionic_amd64__binary/
[py-trees-ros-viewer-build-status-eloquent-image]: http://build.ros2.org/job/Ebin_uB64__py_trees_ros_viewer__ubuntu_bionic_amd64__binary/badge/icon?style=plastic
[py-trees-ros-viewer-build-status-dashing]: http://build.ros2.org/job/Dbin_uB64__py_trees_ros_viewer__ubuntu_bionic_amd64__binary/
[py-trees-ros-viewer-build-status-dashing-image]: http://build.ros2.org/job/Dbin_uB64__py_trees_ros_viewer__ubuntu_bionic_amd64__binary/badge/icon?style=plastic
[py-trees-ros-viewer-docs-0.2.x]: https://github.com/splintered-reality/py_trees_ros_viewer/blob/release/0.2.x/README.md
[py-trees-ros-viewer-docs-0.1.x]: https://github.com/splintered-reality/py_trees_ros_viewer/blob/release/0.1.x/README.md
[py-trees-ros-viewer-sources-0.2.x]: https://github.com/splintered-reality/py_trees_ros_viewer/tree/release/0.2.x
[py-trees-ros-viewer-sources-0.1.x]: https://github.com/splintered-reality/py_trees_ros_viewer/tree/release/0.1.x
[py-trees-ros-viewer-ros-index]: https://index.ros.org/p/py_trees_ros_viewer/github-splintered-reality-py_trees_ros_viewer

[py-trees-msgs-build-status-kinetic]: http://build.ros.org/job/Kbin_uX64__py_trees_msgs__ubuntu_xenial_amd64__binary
[py-trees-msgs-build-status-kinetic-image]: http://build.ros.org/job/Kbin_uX64__py_trees_msgs__ubuntu_xenial_amd64__binary/badge/icon?style=plastic
[py-trees-msgs-build-status-melodic]: http://build.ros.org/job/Mbin_uB64__py_trees_msgs__ubuntu_bionic_amd64__binary
[py-trees-msgs-build-status-melodic-image]: http://build.ros.org/job/Mbin_uB64__py_trees_msgs__ubuntu_bionic_amd64__binary/badge/icon?style=plastic
[py-trees-msgs-docs-kinetic]: http://docs.ros.org/kinetic/api/py_trees_msgs/html/index-msg.html
[py-trees-msgs-docs-kinetic-image]: https://img.shields.io/jenkins/s/http/build.ros.org/job/Kdoc__py_trees_msgs__ubuntu_xenial_amd64.svg?label=docs&style=plastic
[py-trees-msgs-docs-melodic]: http://docs.ros.org/melodic/api/py_trees_msgs/html/index-msg.html
[py-trees-msgs-docs-melodic-image]: https://img.shields.io/jenkins/s/http/build.ros.org/job/Mdoc__py_trees_msgs__ubuntu_bionic_amd64.svg?label=docs&style=plastic
[py-trees-msgs-sources-kinetic]: https://github.com/stonier/py_trees_msgs/tree/release/0.3-kinetic
[py-trees-msgs-sources-melodic]: https://github.com/stonier/py_trees_msgs/tree/release/0.3-melodic
[py-trees-msgs-wiki]: http://wiki.ros.org/py_trees_msgs

[rqt-py-trees-build-status-kinetic]: http://build.ros.org/job/Kbin_uX64__rqt_py_trees__ubuntu_xenial_amd64__binary
[rqt-py-trees-build-status-kinetic-image]: http://build.ros.org/job/Kbin_uX64__rqt_py_trees__ubuntu_xenial_amd64__binary/badge/icon?style=plastic
[rqt-py-trees-build-status-melodic]: http://build.ros.org/job/Mbin_uB64__rqt_py_trees__ubuntu_bionic_amd64__binary
[rqt-py-trees-build-status-melodic-image]: http://build.ros.org/job/Mbin_uB64__rqt_py_trees__ubuntu_bionic_amd64__binary/badge/icon?style=plastic
[rqt-py-trees-sources-kinetic]: https://github.com/splintered-reality/rqt_py_trees/tree/release/0.3-kinetic
[rqt-py-trees-sources-melodic]: https://github.com/splintered-reality/rqt_py_trees/tree/release/0.3-melodic
[rqt-py-trees-wiki]: http://wiki.ros.org/rqt_py_trees
