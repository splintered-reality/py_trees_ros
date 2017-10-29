I can never remember how to run them...

* http://wiki.ros.org/rostest/Commandline

```
rostest --text mytest.test
```

To connect to the default roscore port (so that you can introspect / separate nodes etc):

```
# in a first shell
roscore
# in a second shell (reuse the current roscore and clear parameters)
rostest -r -c --text mytest.test
```
