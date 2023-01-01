# Development Environment

## Requirements

This assumes the host is running an XOrg server (e.g. Ubuntu). This is relayed to the
devcontainer to run the qt viwers. You may need a different devcontainer configuration if running,
e.g. a Wayland server.

## Sources

```
# Fetch sources
$ mkdir src
$ wget -O - https://raw.githubusercontent.com/stonier/repos_index/devel/foxy/py_trees.repos | vcs import ./src

# Open VSCode
$ code ./src/py_trees_ros
```

## VSCode DevContainer

At this point you can use VSCode to "Re-open project in container" -> this will bring everything from
the root of the colcon workspace (i.e. below `./src`) into both the devcontainer and the vscode UI.
