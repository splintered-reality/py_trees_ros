# Tests

Make sure you source the environment to run tests.

## Executing Tests

```bash
# run all tests will full stdout
$ python3 -m unittest discover
# run a single test
$ cd tests && python3 ./test_action_client.py
# step back and run from setup.py (what colcon does)
$ python3 setup.py test
```

