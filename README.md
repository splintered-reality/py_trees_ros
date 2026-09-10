# PyTrees for ROS

Behaviours, trees, and utilities that extend [PyTrees](https://github.com/splintered-reality/py_trees) for use with ROS.

![Trees](docs/sources/images/trees.png?raw=true "Behaviour Trees")

## Getting Started

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

| ROS Distro | [Rolling][rolling-build-farm] | [Lyrical][lyrical-build-farm] | [Kilted][kilted-build-farm] | [Jazzy][jazzy-build-farm] | [Humble][humble-build-farm] |
|:---:|:---:|:---:|:---:|:---:|:---:|
| [py_trees][py-trees-ros-index] | [![2.6.x][2.6.x-sources-image]][py-trees-sources-2.6.x]<br/>[![Build Status][py-trees-build-status-rolling-image]][py-trees-build-status-rolling]<br/>[![2.6.x-Docs][2.6.x-rtd-image]][py-trees-docs-2.6.x] | [![2.6.x][2.6.x-sources-image]][py-trees-sources-2.6.x]<br/>[![Build Status][py-trees-build-status-lyrical-image]][py-trees-build-status-lyrical]<br/>[![2.6.x-Docs][2.6.x-rtd-image]][py-trees-docs-2.6.x] | [![2.6.x][2.6.x-sources-image]][py-trees-sources-2.6.x]<br/>[![Build Status][py-trees-build-status-kilted-image]][py-trees-build-status-kilted]<br/>[![2.6.x-Docs][2.6.x-rtd-image]][py-trees-docs-2.6.x] | [![2.6.x][2.6.x-sources-image]][py-trees-sources-2.6.x]<br/>[![Build Status][py-trees-build-status-jazzy-image]][py-trees-build-status-jazzy]<br/>[![2.6.x-Docs][2.6.x-rtd-image]][py-trees-docs-2.6.x] | [![2.6.x][2.6.x-sources-image]][py-trees-sources-2.6.x]<br/>[![Build Status][py-trees-build-status-humble-image]][py-trees-build-status-humble]<br/>[![2.6.x-Docs][2.6.x-rtd-image]][py-trees-docs-2.6.x] |
| [py_trees_ros_interfaces][py-trees-ros-interfaces-ros-index] | [![2.1.x][2.1.x-sources-image]][py-trees-ros-interfaces-sources-2.1.x]<br/>[![Build Status][py-trees-ros-interfaces-build-status-rolling-image]][py-trees-ros-interfaces-build-status-rolling] | [![2.1.x][2.1.x-sources-image]][py-trees-ros-interfaces-sources-2.1.x]<br/>[![Build Status][py-trees-ros-interfaces-build-status-lyrical-image]][py-trees-ros-interfaces-build-status-lyrical] | [![2.1.x][2.1.x-sources-image]][py-trees-ros-interfaces-sources-2.1.x]<br/>[![Build Status][py-trees-ros-interfaces-build-status-kilted-image]][py-trees-ros-interfaces-build-status-kilted] | [![2.1.x][2.1.x-sources-image]][py-trees-ros-interfaces-sources-2.1.x]<br/>[![Build Status][py-trees-ros-interfaces-build-status-jazzy-image]][py-trees-ros-interfaces-build-status-jazzy] | [![2.1.x][2.1.x-sources-image]][py-trees-ros-interfaces-sources-2.1.x]<br/>[![Build Status][py-trees-ros-interfaces-build-status-humble-image]][py-trees-ros-interfaces-build-status-humble] |
| [py_trees_ros][py-trees-ros-ros-index] | [![2.6.x][2.6.x-sources-image]][py-trees-ros-sources-2.6.x]<br/>[![Build Status][py-trees-ros-build-status-rolling-image]][py-trees-ros-build-status-rolling]<br/>[![2.6.x-Docs][2.6.x-rtd-image]][py-trees-ros-docs-2.6.x] | [![2.6.x][2.6.x-sources-image]][py-trees-ros-sources-2.6.x]<br/>[![Build Status][py-trees-ros-build-status-lyrical-image]][py-trees-ros-build-status-lyrical]<br/>[![2.6.x-Docs][2.6.x-rtd-image]][py-trees-ros-docs-2.6.x] | [![2.6.x][2.6.x-sources-image]][py-trees-ros-sources-2.6.x]<br/>[![Build Status][py-trees-ros-build-status-kilted-image]][py-trees-ros-build-status-kilted]<br/>[![2.6.x-Docs][2.6.x-rtd-image]][py-trees-ros-docs-2.6.x] | [![2.6.x][2.6.x-sources-image]][py-trees-ros-sources-2.6.x]<br/>[![Build Status][py-trees-ros-build-status-jazzy-image]][py-trees-ros-build-status-jazzy]<br/>[![2.6.x-Docs][2.6.x-rtd-image]][py-trees-ros-docs-2.6.x] | [![2.6.x][2.6.x-sources-image]][py-trees-ros-sources-2.6.x]<br/>[![Build Status][py-trees-ros-build-status-humble-image]][py-trees-ros-build-status-humble]<br/>[![2.6.x-Docs][2.6.x-rtd-image]][py-trees-ros-docs-2.6.x] |
| [py_trees_ros_tutorials][py-trees-ros-tutorials-ros-index] | [![2.6.x][2.6.x-sources-image]][py-trees-ros-tutorials-sources-2.6.x]<br/>[![Build Status][py-trees-ros-tutorials-build-status-rolling-image]][py-trees-ros-tutorials-build-status-rolling]<br/>[![2.6.x-Docs][2.6.x-rtd-image]][py-trees-ros-tutorials-docs-2.6.x] | [![2.6.x][2.6.x-sources-image]][py-trees-ros-tutorials-sources-2.6.x]<br/>[![Build Status][py-trees-ros-tutorials-build-status-lyrical-image]][py-trees-ros-tutorials-build-status-lyrical]<br/>[![2.6.x-Docs][2.6.x-rtd-image]][py-trees-ros-tutorials-docs-2.6.x] | [![2.6.x][2.6.x-sources-image]][py-trees-ros-tutorials-sources-2.6.x]<br/>[![Build Status][py-trees-ros-tutorials-build-status-kilted-image]][py-trees-ros-tutorials-build-status-kilted]<br/>[![2.6.x-Docs][2.6.x-rtd-image]][py-trees-ros-tutorials-docs-2.6.x] | [![2.6.x][2.6.x-sources-image]][py-trees-ros-tutorials-sources-2.6.x]<br/>[![Build Status][py-trees-ros-tutorials-build-status-jazzy-image]][py-trees-ros-tutorials-build-status-jazzy]<br/>[![2.6.x-Docs][2.6.x-rtd-image]][py-trees-ros-tutorials-docs-2.6.x] | [![2.6.x][2.6.x-sources-image]][py-trees-ros-tutorials-sources-2.6.x]<br/>[![Build Status][py-trees-ros-tutorials-build-status-humble-image]][py-trees-ros-tutorials-build-status-humble]<br/>[![2.6.x-Docs][2.6.x-rtd-image]][py-trees-ros-tutorials-docs-2.6.x] |
| [py_trees_js][py-trees-js-ros-index] | [![0.6.x][0.6.x-sources-image]][py-trees-js-sources-0.6.x]<br/>[![Build Status][py-trees-js-build-status-rolling-image]][py-trees-js-build-status-rolling]<br/>[![0.6.x-Docs][readme-docs-image]][py-trees-js-docs-0.6.x] | [![0.6.x][0.6.x-sources-image]][py-trees-js-sources-0.6.x]<br/>[![Build Status][py-trees-js-build-status-lyrical-image]][py-trees-js-build-status-lyrical]<br/>[![0.6.x-Docs][readme-docs-image]][py-trees-js-docs-0.6.x] | [![0.6.x][0.6.x-sources-image]][py-trees-js-sources-0.6.x]<br/>[![Build Status][py-trees-js-build-status-kilted-image]][py-trees-js-build-status-kilted]<br/>[![0.6.x-Docs][readme-docs-image]][py-trees-js-docs-0.6.x] | [![0.6.x][0.6.x-sources-image]][py-trees-js-sources-0.6.x]<br/>[![Build Status][py-trees-js-build-status-jazzy-image]][py-trees-js-build-status-jazzy]<br/>[![0.6.x-Docs][readme-docs-image]][py-trees-js-docs-0.6.x] | [![0.6.x][0.6.x-sources-image]][py-trees-js-sources-0.6.x]<br/>[![Build Status][py-trees-js-build-status-humble-image]][py-trees-js-build-status-humble]<br/>[![0.6.x-Docs][readme-docs-image]][py-trees-js-docs-0.6.x] | 
| [py_trees_ros_viewer][py-trees-ros-viewer-ros-index] | [![0.3.x][0.3.x-sources-image]][py-trees-ros-viewer-sources-0.3.x]<br/>[![Build Status][py-trees-ros-viewer-build-status-rolling-image]][py-trees-ros-viewer-build-status-rolling]<br/>[![0.3.x-Docs][readme-docs-image]][py-trees-ros-viewer-docs-0.3.x] | [![0.3.x][0.3.x-sources-image]][py-trees-ros-viewer-sources-0.3.x]<br/>[![Build Status][py-trees-ros-viewer-build-status-lyrical-image]][py-trees-ros-viewer-build-status-lyrical]<br/>[![0.3.x-Docs][readme-docs-image]][py-trees-ros-viewer-docs-0.3.x] | [![0.3.x][0.3.x-sources-image]][py-trees-ros-viewer-sources-0.3.x]<br/>[![Build Status][py-trees-ros-viewer-build-status-kilted-image]][py-trees-ros-viewer-build-status-kilted]<br/>[![0.3.x-Docs][readme-docs-image]][py-trees-ros-viewer-docs-0.3.x] | [![0.3.x][0.3.x-sources-image]][py-trees-ros-viewer-sources-0.3.x]<br/>[![Build Status][py-trees-ros-viewer-build-status-jazzy-image]][py-trees-ros-viewer-build-status-jazzy]<br/>[![0.3.x-Docs][readme-docs-image]][py-trees-ros-viewer-docs-0.3.x] | [![0.3.x][0.3.x-sources-image]][py-trees-ros-viewer-sources-0.3.x]<br/>[![Build Status][py-trees-ros-viewer-build-status-humble-image]][py-trees-ros-viewer-build-status-humble]<br/>[![0.3.x-Docs][readme-docs-image]][py-trees-ros-viewer-docs-0.3.x] |

[2.6.x-sources-image]: http://img.shields.io/badge/sources-2.6.x-blue.svg?style=plastic
[2.1.x-sources-image]: http://img.shields.io/badge/sources-2.1.x-blue.svg?style=plastic
[0.6.x-sources-image]: http://img.shields.io/badge/sources-0.6.x-blue.svg?style=plastic
[0.3.x-sources-image]: http://img.shields.io/badge/sources-0.3.x-blue.svg?style=plastic

[2.6.x-rtd-image]: https://readthedocs.org/projects/py-trees/badge/?version=release-2.6.x&style=plastic

[readme-docs-image]: http://img.shields.io/badge/docs-README-brightgreen.svg?style=plastic

[rolling-build-farm]: http://repo.ros2.org/status_page/ros_rolling_default.html?q=py_trees
[lyrical-build-farm]: http://repo.ros2.org/status_page/ros_lyrical_default.html?q=py_trees
[kilted-build-farm]: http://repo.ros2.org/status_page/ros_kilted_default.html?q=py_trees
[jazzy-build-farm]: http://repo.ros2.org/status_page/ros_jazzy_default.html?q=py_trees
[humble-build-farm]: http://repo.ros2.org/status_page/ros_humble_default.html?q=py_trees

[py-trees-build-status-rolling]: https://build.ros2.org/job/Rbin_uR64__py_trees__ubuntu_resolute_amd64__binary/
[py-trees-build-status-rolling-image]: https://build.ros2.org/job/Rbin_uR64__py_trees__ubuntu_resolute_amd64__binary/badge/icon?style=plastic
[py-trees-build-status-lyrical]: https://build.ros2.org/job/Lbin_uR64__py_trees__ubuntu_resolute_amd64__binary/
[py-trees-build-status-lyrical-image]: https://build.ros2.org/job/Lbin_uR64__py_trees__ubuntu_resolute_amd64__binary/badge/icon?style=plastic
[py-trees-build-status-kilted]: https://build.ros2.org/job/Kbin_uN64__py_trees__ubuntu_noble_amd64__binary/
[py-trees-build-status-kilted-image]: https://build.ros2.org/job/Kbin_uN64__py_trees__ubuntu_noble_amd64__binary/badge/icon?style=plastic
[py-trees-build-status-jazzy]: https://build.ros2.org/job/Jbin_uN64__py_trees__ubuntu_noble_amd64__binary/
[py-trees-build-status-jazzy-image]: https://build.ros2.org/job/Jbin_uN64__py_trees__ubuntu_noble_amd64__binary/badge/icon?style=plastic
[py-trees-build-status-humble]: https://build.ros2.org/job/Hbin_uJ64__py_trees__ubuntu_jammy_amd64__binary/
[py-trees-build-status-humble-image]: https://build.ros2.org/job/Hbin_uJ64__py_trees__ubuntu_jammy_amd64__binary/badge/icon?style=plastic
[py-trees-docs-2.6.x]: http://py-trees.readthedocs.io/en/release-2.6.x/
[py-trees-ros-index]: https://index.ros.org/p/py_trees/
[py-trees-sources-2.6.x]: https://github.com/splintered-reality/py_trees/tree/release/2.6.x

[py-trees-ros-interfaces-build-status-rolling]: https://build.ros2.org/job/Rbin_uR64__py_trees_ros_interfaces__ubuntu_resolute_amd64__binary/
[py-trees-ros-interfaces-build-status-rolling-image]: https://build.ros2.org/job/Rbin_uR64__py_trees_ros_interfaces__ubuntu_resolute_amd64__binary/badge/icon?style=plastic
[py-trees-ros-interfaces-build-status-lyrical]: https://build.ros2.org/job/Lbin_uR64__py_trees_ros_interfaces__ubuntu_resolute_amd64__binary/
[py-trees-ros-interfaces-build-status-lyrical-image]: https://build.ros2.org/job/Lbin_uR64__py_trees_ros_interfaces__ubuntu_resolute_amd64__binary/badge/icon?style=plastic
[py-trees-ros-interfaces-build-status-kilted]: https://build.ros2.org/job/Kbin_uN64__py_trees_ros_interfaces__ubuntu_noble_amd64__binary/
[py-trees-ros-interfaces-build-status-kilted-image]: https://build.ros2.org/job/Kbin_uN64__py_trees_ros_interfaces__ubuntu_noble_amd64__binary/badge/icon?style=plastic
[py-trees-ros-interfaces-build-status-jazzy]: https://build.ros2.org/job/Jbin_uN64__py_trees_ros_interfaces__ubuntu_noble_amd64__binary/
[py-trees-ros-interfaces-build-status-jazzy-image]: https://build.ros2.org/job/Jbin_uN64__py_trees_ros_interfaces__ubuntu_noble_amd64__binary/badge/icon?style=plastic
[py-trees-ros-interfaces-build-status-humble]: https://build.ros2.org/job/Hbin_uJ64__py_trees_ros_interfaces__ubuntu_jammy_amd64__binary/
[py-trees-ros-interfaces-build-status-humble-image]: https://build.ros2.org/job/Hbin_uJ64__py_trees_ros_interfaces__ubuntu_jammy_amd64__binary/badge/icon?style=plastic
[py-trees-ros-interfaces-ros-index]: https://index.ros.org/p/py_trees_ros_interfaces/
[py-trees-ros-interfaces-sources-2.1.x]: https://github.com/splintered-reality/py_trees_ros_interfaces/tree/release/2.1.x

[py-trees-ros-build-status-rolling]: https://build.ros2.org/job/Rbin_uR64__py_trees_ros__ubuntu_resolute_amd64__binary/
[py-trees-ros-build-status-rolling-image]: https://build.ros2.org/job/Rbin_uR64__py_trees_ros__ubuntu_resolute_amd64__binary/badge/icon?style=plastic
[py-trees-ros-build-status-lyrical]: https://build.ros2.org/job/Lbin_uR64__py_trees_ros__ubuntu_resolute_amd64__binary/
[py-trees-ros-build-status-lyrical-image]: https://build.ros2.org/job/Lbin_uR64__py_trees_ros__ubuntu_resolute_amd64__binary/badge/icon?style=plastic
[py-trees-ros-build-status-kilted]: https://build.ros2.org/job/Kbin_uN64__py_trees_ros__ubuntu_noble_amd64__binary/
[py-trees-ros-build-status-kilted-image]: https://build.ros2.org/job/Kbin_uN64__py_trees_ros__ubuntu_noble_amd64__binary/badge/icon?style=plastic
[py-trees-ros-build-status-jazzy]: https://build.ros2.org/job/Jbin_uN64__py_trees_ros__ubuntu_noble_amd64__binary/
[py-trees-ros-build-status-jazzy-image]: https://build.ros2.org/job/Jbin_uN64__py_trees_ros__ubuntu_noble_amd64__binary/badge/icon?style=plastic
[py-trees-ros-build-status-humble]: https://build.ros2.org/job/Hbin_uJ64__py_trees_ros__ubuntu_jammy_amd64__binary/
[py-trees-ros-build-status-humble-image]: https://build.ros2.org/job/Hbin_uJ64__py_trees_ros__ubuntu_jammy_amd64__binary/badge/icon?style=plastic
[py-trees-ros-docs-2.6.x]: http://py-trees-ros.readthedocs.io/en/release-2.6.x/
[py-trees-ros-ros-index]: https://index.ros.org/p/py_trees_ros/
[py-trees-ros-sources-2.6.x]: https://github.com/splintered-reality/py_trees_ros/tree/release/2.6.x

[py-trees-ros-tutorials-build-status-rolling]: http://build.ros2.org/job/Rbin_uR64__py_trees_ros_tutorials__ubuntu_resolute_amd64__binary/
[py-trees-ros-tutorials-build-status-rolling-image]: http://build.ros2.org/job/Rbin_uR64__py_trees_ros_tutorials__ubuntu_resolute_amd64__binary/badge/icon?style=plastic
[py-trees-ros-tutorials-build-status-lyrical]: http://build.ros2.org/job/Lbin_uR64__py_trees_ros_tutorials__ubuntu_resolute_amd64__binary/
[py-trees-ros-tutorials-build-status-lyrical-image]: http://build.ros2.org/job/Lbin_uR64__py_trees_ros_tutorials__ubuntu_resolute_amd64__binary/badge/icon?style=plastic
[py-trees-ros-tutorials-build-status-kilted]: http://build.ros2.org/job/Kbin_uN64__py_trees_ros_tutorials__ubuntu_noble_amd64__binary/
[py-trees-ros-tutorials-build-status-kilted-image]: http://build.ros2.org/job/Kbin_uN64__py_trees_ros_tutorials__ubuntu_noble_amd64__binary/badge/icon?style=plastic
[py-trees-ros-tutorials-build-status-jazzy]: http://build.ros2.org/job/Jbin_uN64__py_trees_ros_tutorials__ubuntu_noble_amd64__binary/
[py-trees-ros-tutorials-build-status-jazzy-image]: http://build.ros2.org/job/Jbin_uN64__py_trees_ros_tutorials__ubuntu_noble_amd64__binary/badge/icon?style=plastic
[py-trees-ros-tutorials-build-status-humble]: http://build.ros2.org/job/Hbin_uJ64__py_trees_ros_tutorials__ubuntu_jammy_amd64__binary/
[py-trees-ros-tutorials-build-status-humble-image]: http://build.ros2.org/job/Hbin_uJ64__py_trees_ros_tutorials__ubuntu_jammy_amd64__binary/badge/icon?style=plastic
[py-trees-ros-tutorials-docs-2.6.x]: http://py-trees-ros-tutorials.readthedocs.io/en/release-2.6.x/
[py-trees-ros-tutorials-sources-2.6.x]: https://github.com/splintered-reality/py_trees_ros_tutorials/tree/release/2.6.x
[py-trees-ros-tutorials-ros-index]: https://index.ros.org/p/py_trees_ros_tutorials/

[py-trees-js-build-status-rolling]: https://build.ros2.org/job/Rbin_uR64__py_trees_js__ubuntu_resolute_amd64__binary/
[py-trees-js-build-status-rolling-image]: https://build.ros2.org/job/Rbin_uR64__py_trees_js__ubuntu_resolute_amd64__binary/badge/icon?style=plastic
[py-trees-js-build-status-lyrical]: https://build.ros2.org/job/Lbin_uR64__py_trees_js__ubuntu_resolute_amd64__binary/
[py-trees-js-build-status-lyrical-image]: https://build.ros2.org/job/Lbin_uR64__py_trees_js__ubuntu_resolute_amd64__binary/badge/icon?style=plastic
[py-trees-js-build-status-kilted]: https://build.ros2.org/job/Kbin_uN64__py_trees_js__ubuntu_noble_amd64__binary/
[py-trees-js-build-status-kilted-image]: https://build.ros2.org/job/Kbin_uN64__py_trees_js__ubuntu_noble_amd64__binary/badge/icon?style=plastic
[py-trees-js-build-status-jazzy]: https://build.ros2.org/job/Jbin_uN64__py_trees_js__ubuntu_noble_amd64__binary/
[py-trees-js-build-status-jazzy-image]: https://build.ros2.org/job/Jbin_uN64__py_trees_js__ubuntu_noble_amd64__binary/badge/icon?style=plastic
[py-trees-js-build-status-humble]: https://build.ros2.org/job/Hbin_uJ64__py_trees_js__ubuntu_jammy_amd64__binary/
[py-trees-js-build-status-humble-image]: https://build.ros2.org/job/Hbin_uJ64__py_trees_js__ubuntu_jammy_amd64__binary/badge/icon?style=plastic
[py-trees-js-docs-0.6.x]: https://github.com/splintered-reality/py_trees_js/blob/release/0.6.x/README.md
[py-trees-js-sources-0.6.x]: https://github.com/splintered-reality/py_trees_js/tree/release/0.6.x
[py-trees-js-ros-index]: https://index.ros.org/p/py_trees_js/

[py-trees-ros-viewer-build-status-rolling]: https://build.ros2.org/job/Rbin_uR64__py_trees_ros_viewer__ubuntu_resolute_amd64__binary/
[py-trees-ros-viewer-build-status-rolling-image]: https://build.ros2.org/job/Rbin_uR64__py_trees_ros_viewer__ubuntu_resolute_amd64__binary/badge/icon?style=plastic
[py-trees-ros-viewer-build-status-lyrical]: https://build.ros2.org/job/Lbin_uR64__py_trees_ros_viewer__ubuntu_resolute_amd64__binary/
[py-trees-ros-viewer-build-status-lyrical-image]: https://build.ros2.org/job/Lbin_uR64__py_trees_ros_viewer__ubuntu_resolute_amd64__binary/badge/icon?style=plastic
[py-trees-ros-viewer-build-status-kilted]: https://build.ros2.org/job/Kbin_uN64__py_trees_ros_viewer__ubuntu_noble_amd64__binary/
[py-trees-ros-viewer-build-status-kilted-image]: https://build.ros2.org/job/Kbin_uN64__py_trees_ros_viewer__ubuntu_noble_amd64__binary/badge/icon?style=plastic
[py-trees-ros-viewer-build-status-jazzy]: https://build.ros2.org/job/Jbin_uN64__py_trees_ros_viewer__ubuntu_noble_amd64__binary/
[py-trees-ros-viewer-build-status-jazzy-image]: https://build.ros2.org/job/Jbin_uN64__py_trees_ros_viewer__ubuntu_noble_amd64__binary/badge/icon?style=plastic
[py-trees-ros-viewer-build-status-humble]: https://build.ros2.org/job/Hbin_uJ64__py_trees_ros_viewer__ubuntu_jammy_amd64__binary/
[py-trees-ros-viewer-build-status-humble-image]: https://build.ros2.org/job/Hbin_uJ64__py_trees_ros_viewer__ubuntu_jammy_amd64__binary/badge/icon?style=plastic
[py-trees-ros-viewer-docs-0.3.x]: https://github.com/splintered-reality/py_trees_ros_viewer/blob/release/0.3.x/README.md
[py-trees-ros-viewer-sources-0.3.x]: https://github.com/splintered-reality/py_trees_ros_viewer/tree/release/0.3.x
[py-trees-ros-viewer-ros-index]: https://index.ros.org/p/py_trees_ros_viewer/
