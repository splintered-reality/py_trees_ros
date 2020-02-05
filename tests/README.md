# Executing Tests

```bash
# run all tests in the current directory
$ pytest-3

# run all tests with full stdout (-s / --capture=no)
$ pytest-3 -s

# run a test module
$ pytest-3 -s test_alakazam.py

# run a single test
$ pytest-3 -s test_action_clients.py::test_success

# run using setuptools
$ python3 setup.py test
```
