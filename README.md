# PyTrees for ROS

Behaviours, trees, and utilities that extend [PyTrees](https://github.com/splintered-reality/py_trees) for use with ROS.

![Trees](docs/images/trees.png?raw=true "Behaviour Trees")

## Getting Started (ROS 2)

Choose your ROS distro and install via debians, e.g., for Jazzy:
```
$ sudo apt install \
    ros-jazzy-py-trees \
    ros-jazzy-py-trees-ros-interfaces \
    ros-jazzy-py-trees-ros \
    ros-jazzy-py-trees-ros-tutorials \
    ros-jazzy-py-trees-ros-viewer
```

Dive into the [PyTrees Docs](https://py-trees.readthedocs.io/en/devel/) for a basic primer (non-ROS) on behaviour trees, then move on to 
the [PyTrees ROS Tutorials](https://py-trees-ros-tutorials.readthedocs.io/en/devel/) which incrementally build a scenario for a robot.
The [PyTrees ROS](https://py-trees-ros.readthedocs.io/en/devel/) documentation provides API docs for ROS specific trees, idioms, and behaviours respectively.

For version specific releases of the documentation, refer to the documentation links in the matrix below.

| ROS 2 | [Rolling][rolling-build-farm] | [Jazzy][jazzy-build-farm] | [Humble][humble-build-farm] |
|:---:|:---:|:---:|:---:|
| [py_trees][py-trees-ros-index] | [![2.3.x][2.3.x-sources-image]][py-trees-sources-2.3.x]<br/>[![Build Status][py-trees-build-status-rolling-image]][py-trees-build-status-rolling]<br/>[![2.3.x-Docs][2.3.x-rtd-image]][py-trees-docs-2.3.x] | [![2.3.x][2.3.x-sources-image]][py-trees-sources-2.3.x]<br/>[![Build Status][py-trees-build-status-jazzy-image]][py-trees-build-status-jazzy]<br/>[![2.3.x-Docs][2.3.x-rtd-image]][py-trees-docs-2.3.x] | [![2.3.x][2.3.x-sources-image]][py-trees-sources-2.3.x]<br/>[![Build Status][py-trees-build-status-humble-image]][py-trees-build-status-humble]<br/>[![2.3.x-Docs][2.3.x-rtd-image]][py-trees-docs-2.3.x] |
| [py_trees_ros_interfaces][py-trees-ros-interfaces-ros-index] | [![2.3.x][2.3.x-sources-image]][py-trees-ros-interfaces-sources-2.3.x]<br/>[![Build Status][py-trees-ros-interfaces-build-status-rolling-image]][py-trees-ros-interfaces-build-status-rolling] | [![2.3.x][2.3.x-sources-image]][py-trees-ros-interfaces-sources-2.3.x]<br/>[![Build Status][py-trees-ros-interfaces-build-status-jazzy-image]][py-trees-ros-interfaces-build-status-jazzy] | [![2.3.x][2.3.x-sources-image]][py-trees-ros-interfaces-sources-2.3.x]<br/>[![Build Status][py-trees-ros-interfaces-build-status-humble-image]][py-trees-ros-interfaces-build-status-humble] |
| [py_trees_ros][py-trees-ros-ros-index] | [![2.3.x][2.3.x-sources-image]][py-trees-ros-sources-2.3.x]<br/>[![Build Status][py-trees-ros-build-status-rolling-image]][py-trees-ros-build-status-rolling]<br/>[![2.3.x-Docs][2.3.x-rtd-image]][py-trees-ros-docs-2.3.x] | [![2.3.x][2.3.x-sources-image]][py-trees-ros-sources-2.3.x]<br/>[![Build Status][py-trees-ros-build-status-jazzy-image]][py-trees-ros-build-status-jazzy]<br/>[![2.3.x-Docs][2.3.x-rtd-image]][py-trees-ros-docs-2.3.x] | [![2.3.x][2.3.x-sources-image]][py-trees-ros-sources-2.3.x]<br/>[![Build Status][py-trees-ros-build-status-humble-image]][py-trees-ros-build-status-humble]<br/>[![2.3.x-Docs][2.3.x-rtd-image]][py-trees-ros-docs-2.3.x] |
| [py_trees_ros_tutorials][py-trees-ros-tutorials-ros-index] | [![2.3.x][2.3.x-sources-image]][py-trees-ros-tutorials-sources-2.3.x]<br/>[![Build Status][py-trees-ros-tutorials-build-status-rolling-image]][py-trees-ros-tutorials-build-status-rolling]<br/>[![2.3.x-Docs][2.3.x-rtd-image]][py-trees-ros-tutorials-docs-2.3.x] | [![2.3.x][2.3.x-sources-image]][py-trees-ros-tutorials-sources-2.3.x]<br/>[![Build Status][py-trees-ros-tutorials-build-status-jazzy-image]][py-trees-ros-tutorials-build-status-jazzy]<br/>[![2.3.x-Docs][2.3.x-rtd-image]][py-trees-ros-tutorials-docs-2.3.x] | [![2.3.x][2.3.x-sources-image]][py-trees-ros-tutorials-sources-2.3.x]<br/>[![Build Status][py-trees-ros-tutorials-build-status-humble-image]][py-trees-ros-tutorials-build-status-humble]<br/>[![2.3.x-Docs][2.3.x-rtd-image]][py-trees-ros-tutorials-docs-2.3.x] |
| [py_trees_js][py-trees-js-ros-index] | [![0.6.x][0.6.x-sources-image]][py-trees-js-sources-0.6.x]<br/>[![Build Status][py-trees-js-build-status-rolling-image]][py-trees-js-build-status-rolling]<br/>[![0.6.x-Docs][readme-docs-image]][py-trees-js-docs-0.6.x] | [![0.6.x][0.6.x-sources-image]][py-trees-js-sources-0.6.x]<br/>[![Build Status][py-trees-js-build-status-jazzy-image]][py-trees-js-build-status-jazzy]<br/>[![0.6.x-Docs][readme-docs-image]][py-trees-js-docs-0.6.x] | [![0.6.x][0.6.x-sources-image]][py-trees-js-sources-0.6.x]<br/>[![Build Status][py-trees-js-build-status-humble-image]][py-trees-js-build-status-humble]<br/>[![0.6.x-Docs][readme-docs-image]][py-trees-js-docs-0.6.x] | 
| [py_trees_ros_viewer][py-trees-ros-viewer-ros-index] | [![0.2.x][0.2.x-sources-image]][py-trees-ros-viewer-sources-0.2.x]<br/>[![Build Status][py-trees-ros-viewer-build-status-rolling-image]][py-trees-ros-viewer-build-status-rolling]<br/>[![0.2.x-Docs][readme-docs-image]][py-trees-ros-viewer-docs-0.2.x] | [![0.2.x][0.2.x-sources-image]][py-trees-ros-viewer-sources-0.2.x]<br/>[![Build Status][py-trees-ros-viewer-build-status-jazzy-image]][py-trees-ros-viewer-build-status-jazzy]<br/>[![0.2.x-Docs][readme-docs-image]][py-trees-ros-viewer-docs-0.2.x] | [![0.2.x][0.2.x-sources-image]][py-trees-ros-viewer-sources-0.2.x]<br/>[![Build Status][py-trees-ros-viewer-build-status-humble-image]][py-trees-ros-viewer-build-status-humble]<br/>[![0.2.x-Docs][readme-docs-image]][py-trees-ros-viewer-docs-0.2.x] |

## Getting Started (ROS 1)

Refer to the documentation links in the matrix below (note: you'll find the tutorials in the `py_trees_ros` documentation).

|  ROS 1 | [Noetic][noetic-build-farm] |
|:---:|:---:|
| [py_trees][py-trees-wiki] | [![0.7.x][0.7.x-sources-image]][py-trees-sources-0.7.x]<br/>[![Build Status][py-trees-build-status-noetic-image]][py-trees-build-status-noetic]<br/>[![Docs Status][py-trees-docs-noetic-image]][py-trees-docs-noetic] |
| [py_trees_msgs][py-trees-msgs-wiki] | [![0.3.x][0.3.x-sources-image]][py-trees-msgs-sources-noetic]<br/>[![Build Status][py-trees-msgs-build-status-noetic-image]][py-trees-msgs-build-status-noetic]<br/>[![Docs Status][py-trees-msgs-docs-noetic-image]][py-trees-msgs-docs-noetic] |
| [py_trees_ros][py-trees-ros-wiki] | [![0.6.x][0.6.x-sources-image]][py-trees-ros-sources-0.6.x]<br/>[![Build Status][py-trees-ros-build-status-noetic-image]][py-trees-ros-build-status-noetic]<br/>[![Docs Status][py-trees-ros-docs-noetic-image]][py-trees-ros-docs-noetic] |
| [rqt_py_trees][rqt-py-trees-wiki] | [![0.4.x][0.4.x-sources-image]][rqt-py-trees-sources-noetic]<br/>[![Build Status][rqt-py-trees-build-status-noetic-image]][rqt-py-trees-build-status-noetic] |


[devel-sources-image]: http://img.shields.io/badge/sources-devel-blue.svg?style=plastic
[2.3.x-sources-image]: http://img.shields.io/badge/sources-2.3.x-blue.svg?style=plastic
[2.2.x-sources-image]: http://img.shields.io/badge/sources-2.2.x-blue.svg?style=plastic
[2.1.x-sources-image]: http://img.shields.io/badge/sources-2.1.x-blue.svg?style=plastic
[2.0.x-sources-image]: http://img.shields.io/badge/sources-2.0.x-blue.svg?style=plastic
[1.3.x-sources-image]: http://img.shields.io/badge/sources-1.3.x-blue.svg?style=plastic
[1.2.x-sources-image]: http://img.shields.io/badge/sources-1.2.x-blue.svg?style=plastic
[1.1.x-sources-image]: http://img.shields.io/badge/sources-1.1.x-blue.svg?style=plastic
[1.0.x-sources-image]: http://img.shields.io/badge/sources-1.0.x-blue.svg?style=plastic
[0.7.x-sources-image]: http://img.shields.io/badge/sources-0.7.x-blue.svg?style=plastic
[0.6.x-sources-image]: http://img.shields.io/badge/sources-0.6.x-blue.svg?style=plastic
[0.5.x-sources-image]: http://img.shields.io/badge/sources-0.5.x-blue.svg?style=plastic
[0.4.x-sources-image]: http://img.shields.io/badge/sources-0.4.x-blue.svg?style=plastic
[0.3.x-sources-image]: http://img.shields.io/badge/sources-0.3.x-blue.svg?style=plastic
[0.2.x-sources-image]: http://img.shields.io/badge/sources-0.2.x-blue.svg?style=plastic
[0.1.x-sources-image]: http://img.shields.io/badge/sources-0.1.x-blue.svg?style=plastic

[devel-rtd-image]: https://readthedocs.org/projects/py-trees/badge/?version=devel&style=plastic
[2.3.x-rtd-image]: https://readthedocs.org/projects/py-trees/badge/?version=release-2.3.x&style=plastic
[2.2.x-rtd-image]: https://readthedocs.org/projects/py-trees/badge/?version=release-2.2.x&style=plastic
[2.1.x-rtd-image]: https://readthedocs.org/projects/py-trees/badge/?version=release-2.1.x&style=plastic
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

[rolling-build-farm]: http://repo.ros2.org/status_page/ros_rolling_default.html?q=py_trees
[jazzy-build-farm]: http://repo.ros2.org/status_page/ros_jazzy_default.html?q=py_trees
[humble-build-farm]: http://repo.ros2.org/status_page/ros_humble_default.html?q=py_trees
[noetic-build-farm]: http://repositories.ros.org/status_page/ros_noetic_default.html?q=py_trees

[py-trees-build-status-rolling]: https://build.ros2.org/job/Rbin_uN64__py_trees__ubuntu_noble_amd64__binary/
[py-trees-build-status-rolling-image]: https://build.ros2.org/job/Rbin_uN64__py_trees__ubuntu_noble_amd64__binary/badge/icon?style=plastic
[py-trees-build-status-jazzy]: https://build.ros2.org/job/Jbin_uN64__py_trees__ubuntu_noble_amd64__binary/
[py-trees-build-status-jazzy-image]: https://build.ros2.org/job/Jbin_uN64__py_trees__ubuntu_noble_amd64__binary/badge/icon?style=plastic
[py-trees-build-status-humble]: https://build.ros2.org/job/Hbin_uJ64__py_trees__ubuntu_jammy_amd64__binary/
[py-trees-build-status-humble-image]: https://build.ros2.org/job/Hbin_uJ64__py_trees__ubuntu_jammy_amd64__binary/badge/icon?style=plastic
[py-trees-build-status-noetic]: http://build.ros.org/job/Nbin_uF64__py_trees__ubuntu_focal_amd64__binary
[py-trees-build-status-noetic-image]: http://build.ros.org/job/Nbin_uF64__py_trees__ubuntu_focal_amd64__binary/badge/icon?style=plastic
[py-trees-docs-devel]: http://py-trees.readthedocs.io/
[py-trees-docs-2.3.x]: http://py-trees.readthedocs.io/en/release-2.3.x/
[py-trees-docs-2.2.x]: http://py-trees.readthedocs.io/en/release-2.2.x/
[py-trees-docs-2.1.x]: http://py-trees.readthedocs.io/en/release-2.1.x/
[py-trees-docs-2.0.x]: http://py-trees.readthedocs.io/en/release-2.0.x/
[py-trees-docs-1.3.x]: http://py-trees.readthedocs.io/en/release-1.3.x/
[py-trees-docs-0.7.x]: http://py-trees.readthedocs.io/en/release-0.7.x/
[py-trees-docs-0.6.x]: http://py-trees.readthedocs.io/en/release-0.6.x/
[py-trees-docs-rolling-image]: http://img.shields.io/badge/py_trees-rolling-brightgreen.svg?style=plastic
[py-trees-docs-jazzy-image]: http://img.shields.io/badge/py_trees-jazzy-brightgreen.svg?style=plastic
[py-trees-docs-humble-image]: http://img.shields.io/badge/py_trees-humble-brightgreen.svg?style=plastic
[py-trees-docs-noetic]: http://docs.ros.org/noetic/api/py_trees/html/
[py-trees-docs-noetic-image]: https://img.shields.io/jenkins/s/http/build.ros.org/job/Ndoc__py_trees__ubuntu_focal_amd64.svg?label=docs&style=plastic
[py-trees-ros-index]: https://index.ros.org/p/py_trees/github-splintered-reality-py_trees
[py-trees-sources-devel]: https://github.com/splintered-reality/py_trees/tree/devel
[py-trees-sources-2.3.x]: https://github.com/splintered-reality/py_trees/tree/release/2.3.x
[py-trees-sources-2.2.x]: https://github.com/splintered-reality/py_trees/tree/release/2.2.x
[py-trees-sources-2.1.x]: https://github.com/splintered-reality/py_trees/tree/release/2.1.x
[py-trees-sources-2.0.x]: https://github.com/splintered-reality/py_trees/tree/release/2.0.x
[py-trees-sources-1.3.x]: https://github.com/splintered-reality/py_trees/tree/release/1.3.x
[py-trees-sources-0.7.x]: https://github.com/splintered-reality/py_trees/tree/release/0.7.x
[py-trees-sources-0.6.x]: https://github.com/splintered-reality/py_trees/tree/release/0.6.x
[py-trees-sources-0.5.x]: https://github.com/splintered-reality/py_trees/tree/release/0.5.x
[py-trees-wiki]: http://wiki.ros.org/py_trees

[py-trees-ros-interfaces-build-status-rolling]: https://build.ros2.org/job/Rbin_uN64__py_trees_ros_interfaces__ubuntu_noble_amd64__binary/
[py-trees-ros-interfaces-build-status-rolling-image]: https://build.ros2.org/job/Rbin_uN64__py_trees_ros_interfaces__ubuntu_noble_amd64__binary/badge/icon?style=plastic
[py-trees-ros-interfaces-build-status-jazzy]: https://build.ros2.org/job/Jbin_uN64__py_trees_ros_interfaces__ubuntu_noble_amd64__binary/
[py-trees-ros-interfaces-build-status-jazzy-image]: https://build.ros2.org/job/Jbin_uN64__py_trees_ros_interfaces__ubuntu_noble_amd64__binary/badge/icon?style=plastic
[py-trees-ros-interfaces-build-status-humble]: https://build.ros2.org/job/Hbin_uJ64__py_trees_ros_interfaces__ubuntu_jammy_amd64__binary/
[py-trees-ros-interfaces-build-status-humble-image]: https://build.ros2.org/job/Hbin_uJ64__py_trees_ros_interfaces__ubuntu_jammy_amd64__binary/badge/icon?style=plastic
[py-trees-ros-interfaces-ros-index]: https://index.ros.org/p/py_trees_ros_interfaces/github-splintered-reality-py_trees_ros_interfaces
[py-trees-ros-interfaces-sources-2.3.x]: https://github.com/splintered-reality/py_trees_ros_interfaces/tree/release/2.3.x
[py-trees-ros-interfaces-sources-2.2.x]: https://github.com/splintered-reality/py_trees_ros_interfaces/tree/release/2.2.x
[py-trees-ros-interfaces-sources-2.1.x]: https://github.com/splintered-reality/py_trees_ros_interfaces/tree/release/2.1.x
[py-trees-ros-interfaces-sources-2.0.x]: https://github.com/splintered-reality/py_trees_ros_interfaces/tree/release/2.0.x
[py-trees-ros-interfaces-sources-1.2.x]: https://github.com/splintered-reality/py_trees_ros_interfaces/tree/release/1.2.x
[py-trees-ros-interfaces-sources-1.1.x]: https://github.com/splintered-reality/py_trees_ros_interfaces/tree/release/1.1.x

[py-trees-ros-build-status-rolling]: https://build.ros2.org/job/Rbin_uN64__py_trees_ros__ubuntu_noble_amd64__binary/
[py-trees-ros-build-status-rolling-image]: https://build.ros2.org/job/Rbin_uN64__py_trees_ros__ubuntu_noble_amd64__binary/badge/icon?style=plastic
[py-trees-ros-build-status-jazzy]: https://build.ros2.org/job/Jbin_uN64__py_trees_ros__ubuntu_noble_amd64__binary/
[py-trees-ros-build-status-jazzy-image]: https://build.ros2.org/job/Jbin_uN64__py_trees_ros__ubuntu_noble_amd64__binary/badge/icon?style=plastic
[py-trees-ros-build-status-humble]: https://build.ros2.org/job/Hbin_uJ64__py_trees_ros__ubuntu_jammy_amd64__binary/
[py-trees-ros-build-status-humble-image]: https://build.ros2.org/job/Hbin_uJ64__py_trees_ros__ubuntu_jammy_amd64__binary/badge/icon?style=plastic
[py-trees-ros-build-status-noetic]: http://build.ros.org/job/Nbin_uF64__py_trees_ros__ubuntu_focal_amd64__binary
[py-trees-ros-build-status-noetic-image]: http://build.ros.org/job/Nbin_uF64__py_trees_ros__ubuntu_focal_amd64__binary/badge/icon?style=plastic
[py-trees-ros-docs-2.3.x]: http://py-trees-ros.readthedocs.io/en/release-2.3.x/
[py-trees-ros-docs-2.2.x]: http://py-trees-ros.readthedocs.io/en/release-2.2.x/
[py-trees-ros-docs-2.1.x]: http://py-trees-ros.readthedocs.io/en/release-2.1.x/
[py-trees-ros-docs-2.0.x]: http://py-trees-ros.readthedocs.io/en/release-2.0.x/
[py-trees-ros-docs-1.3.x]: http://py-trees-ros.readthedocs.io/en/release-1.3.x/
[py-trees-ros-docs-1.2.x]: http://py-trees-ros.readthedocs.io/en/release-1.2.x/
[py-trees-ros-docs-noetic]: http://docs.ros.org/noetic/api/py_trees_ros/html/
[py-trees-ros-docs-noetic-image]: https://img.shields.io/jenkins/s/http/build.ros.org/job/Ndoc__py_trees_ros__ubuntu_focal_amd64.svg?label=docs&style=plastic
[py-trees-ros-ros-index]: https://index.ros.org/p/py_trees_ros/github-splintered-reality-py_trees_ros
[py-trees-ros-sources-2.3.x]: https://github.com/splintered-reality/py_trees_ros/tree/release/2.3.x
[py-trees-ros-sources-2.2.x]: https://github.com/splintered-reality/py_trees_ros/tree/release/2.2.x
[py-trees-ros-sources-2.1.x]: https://github.com/splintered-reality/py_trees_ros/tree/release/2.1.x
[py-trees-ros-sources-2.0.x]: https://github.com/splintered-reality/py_trees_ros/tree/release/2.0.x
[py-trees-ros-sources-1.3.x]: https://github.com/splintered-reality/py_trees_ros/tree/release/1.3.x
[py-trees-ros-sources-1.2.x]: https://github.com/splintered-reality/py_trees_ros/tree/release/1.2.x
[py-trees-ros-sources-0.6.x]: https://github.com/splintered-reality/py_trees_ros/tree/release/0.6.x
[py-trees-ros-sources-0.5.x]: https://github.com/splintered-reality/py_trees_ros/tree/release/0.5.x
[py-trees-ros-wiki]: http://wiki.ros.org/py_trees_ros

[py-trees-ros-tutorials-build-status-rolling]: http://build.ros2.org/job/Rbin_uN64__py_trees_ros_tutorials__ubuntu_noble_amd64__binary/
[py-trees-ros-tutorials-build-status-rolling-image]: http://build.ros2.org/job/Rbin_uN64__py_trees_ros_tutorials__ubuntu_noble_amd64__binary/badge/icon?style=plastic
[py-trees-ros-tutorials-build-status-jazzy]: http://build.ros2.org/job/Jbin_uN64__py_trees_ros_tutorials__ubuntu_noble_amd64__binary/
[py-trees-ros-tutorials-build-status-jazzy-image]: http://build.ros2.org/job/Jbin_uN64__py_trees_ros_tutorials__ubuntu_noble_amd64__binary/badge/icon?style=plastic
[py-trees-ros-tutorials-build-status-humble]: http://build.ros2.org/job/Hbin_uJ64__py_trees_ros_tutorials__ubuntu_jammy_amd64__binary/
[py-trees-ros-tutorials-build-status-humble-image]: http://build.ros2.org/job/Hbin_uJ64__py_trees_ros_tutorials__ubuntu_jammy_amd64__binary/badge/icon?style=plastic
[py-trees-ros-tutorials-docs-devel]: http://py-trees-ros-tutorials.readthedocs.io/en/release-devel/
[py-trees-ros-tutorials-docs-2.3.x]: http://py-trees-ros-tutorials.readthedocs.io/en/release-2.3.x/
[py-trees-ros-tutorials-docs-2.1.x]: http://py-trees-ros-tutorials.readthedocs.io/en/release-2.1.x/
[py-trees-ros-tutorials-docs-2.0.x]: http://py-trees-ros-tutorials.readthedocs.io/en/release-2.0.x/
[py-trees-ros-tutorials-docs-1.0.x]: http://py-trees-ros-tutorials.readthedocs.io/en/release-1.0.x/
[py-trees-ros-tutorials-sources-devel]: https://github.com/splintered-reality/py_trees_ros_tutorials/tree/release/devel
[py-trees-ros-tutorials-sources-2.3.x]: https://github.com/splintered-reality/py_trees_ros_tutorials/tree/release/2.3.x
[py-trees-ros-tutorials-sources-2.1.x]: https://github.com/splintered-reality/py_trees_ros_tutorials/tree/release/2.1.x
[py-trees-ros-tutorials-sources-2.0.x]: https://github.com/splintered-reality/py_trees_ros_tutorials/tree/release/2.0.x
[py-trees-ros-tutorials-sources-1.0.x]: https://github.com/splintered-reality/py_trees_ros_tutorials/tree/release/1.0.x
[py-trees-ros-tutorials-ros-index]: https://index.ros.org/p/py_trees_ros_tutorials/github-splintered-reality-py_trees_ros_tutorials

[py-trees-js-build-status-rolling]: https://build.ros2.org/job/Rbin_uN64__py_trees_js__ubuntu_noble_amd64__binary/
[py-trees-js-build-status-rolling-image]: https://build.ros2.org/job/Rbin_uN64__py_trees_js__ubuntu_noble_amd64__binary/badge/icon?style=plastic
[py-trees-js-build-status-jazzy]: https://build.ros2.org/job/Jbin_uN64__py_trees_js__ubuntu_noble_amd64__binary/
[py-trees-js-build-status-jazzy-image]: https://build.ros2.org/job/Jbin_uN64__py_trees_js__ubuntu_noble_amd64__binary/badge/icon?style=plastic
[py-trees-js-build-status-humble]: https://build.ros2.org/job/Hbin_uJ64__py_trees_js__ubuntu_jammy_amd64__binary/
[py-trees-js-build-status-humble-image]: https://build.ros2.org/job/Hbin_uJ64__py_trees_js__ubuntu_jammy_amd64__binary/badge/icon?style=plastic
[py-trees-js-docs-0.6.x]: https://github.com/splintered-reality/py_trees_js/blob/release/0.6.x/README.md
[py-trees-js-sources-0.6.x]: https://github.com/splintered-reality/py_trees_js/tree/release/0.6.x
[py-trees-js-docs-0.5.x]: https://github.com/splintered-reality/py_trees_js/blob/release/0.5.x/README.md
[py-trees-js-sources-0.5.x]: https://github.com/splintered-reality/py_trees_js/tree/release/0.5.x
[py-trees-js-docs-0.4.x]: https://github.com/splintered-reality/py_trees_js/blob/release/0.4.x/README.md
[py-trees-js-sources-0.4.x]: https://github.com/splintered-reality/py_trees_js/tree/release/0.4.x
[py-trees-js-ros-index]: https://index.ros.org/p/py_trees_js/github-splintered-reality-py_trees_js

[py-trees-ros-viewer-build-status-rolling]: https://build.ros2.org/job/Rbin_uN64__py_trees_ros_viewer__ubuntu_noble_amd64__binary/
[py-trees-ros-viewer-build-status-rolling-image]: https://build.ros2.org/job/Rbin_uN64__py_trees_ros_viewer__ubuntu_noble_amd64__binary/badge/icon?style=plastic
[py-trees-ros-viewer-build-status-jazzy]: https://build.ros2.org/job/Jbin_uN64__py_trees_ros_viewer__ubuntu_noble_amd64__binary/
[py-trees-ros-viewer-build-status-jazzy-image]: https://build.ros2.org/job/Jbin_uN64__py_trees_ros_viewer__ubuntu_noble_amd64__binary/badge/icon?style=plastic
[py-trees-ros-viewer-build-status-humble]: https://build.ros2.org/job/Hbin_uJ64__py_trees_ros_viewer__ubuntu_jammy_amd64__binary/
[py-trees-ros-viewer-build-status-humble-image]: https://build.ros2.org/job/Hbin_uJ64__py_trees_ros_viewer__ubuntu_jammy_amd64__binary/badge/icon?style=plastic
[py-trees-ros-viewer-docs-0.2.x]: https://github.com/splintered-reality/py_trees_ros_viewer/blob/release/0.2.x/README.md
[py-trees-ros-viewer-docs-0.1.x]: https://github.com/splintered-reality/py_trees_ros_viewer/blob/release/0.1.x/README.md
[py-trees-ros-viewer-sources-0.2.x]: https://github.com/splintered-reality/py_trees_ros_viewer/tree/release/0.2.x
[py-trees-ros-viewer-sources-0.1.x]: https://github.com/splintered-reality/py_trees_ros_viewer/tree/release/0.1.x
[py-trees-ros-viewer-ros-index]: https://index.ros.org/p/py_trees_ros_viewer/github-splintered-reality-py_trees_ros_viewer

[py-trees-msgs-build-status-noetic]: http://build.ros.org/job/Nbin_uF64__py_trees_msgs__ubuntu_focal_amd64__binary
[py-trees-msgs-build-status-noetic-image]: http://build.ros.org/job/Nbin_uF64__py_trees_msgs__ubuntu_focal_amd64__binary/badge/icon?style=plastic
[py-trees-msgs-docs-noetic]: http://docs.ros.org/noetic/api/py_trees_msgs/html/index-msg.html
[py-trees-msgs-docs-noetic-image]: https://img.shields.io/jenkins/s/http/build.ros.org/job/Ndoc__py_trees_msgs__ubuntu_focal_amd64.svg?label=docs&style=plastic
[py-trees-msgs-sources-noetic]: https://github.com/splintered-reality/py_trees_msgs/tree/release/0.3.x
[py-trees-msgs-wiki]: http://wiki.ros.org/py_trees_msgs

[rqt-py-trees-build-status-noetic]: http://build.ros.org/job/Nbin_uF64__rqt_py_trees__ubuntu_focal_amd64__binary
[rqt-py-trees-build-status-noetic-image]: http://build.ros.org/job/Nbin_uF64__rqt_py_trees__ubuntu_focal_amd64__binary/badge/icon?style=plastic
[rqt-py-trees-sources-noetic]: https://github.com/splintered-reality/rqt_py_trees/tree/release/0.4.x
[rqt-py-trees-wiki]: http://wiki.ros.org/rqt_py_trees
