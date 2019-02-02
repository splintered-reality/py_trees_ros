#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Utilities for the various py-trees ros flavoured programs.
"""

##############################################################################
# Imports
##############################################################################

import py_trees.console as console
import rosservice
import sys

##############################################################################
# Classes
##############################################################################

def discover_namespace(suggested_namespace):
    try:
        service_name_list = rosservice.rosservice_find("py_trees_msgs/OpenBlackboardWatcher")
    except rosservice.ROSServiceIOException as e:
        print(console.red + "ERROR: {0}".format(str(e)) + console.reset)
        sys.exit(1)

    if not service_name_list:
        print(console.red + "ERROR: blackboard services not found" + console.reset)
        sys.exit(1)

    if suggested_namespace is not None:
        for service_name in service_name_list:
            if suggested_namespace in service_name:
                return suggested_namespace
        print("")
        print(console.red
              + "ERROR: blackboard services found {}".format(service_name_list)
              + console.reset
        )
        print(console.red
              + "ERROR: but none matching the requested '{}'".format(suggested_namespace)
              + console.reset)
        print("")
        sys.exit(1)

    if len(service_name_list) > 1:
        print(console.red + "\nERROR: multiple blackboard services found %s" % service_name_list + console.reset)
        print(console.red + "\nERROR: select one with the --namespace argument" + console.reset)
        sys.exit(1)

    # the previous checks guarantee we have a single name here
    service_name = service_name_list[0]
    # drop the last substring
    namespace = '/'.join(service_name.split('/')[:-1])
    return namespace
        
def find_service(namespace, service_type):
    # This method assumes all error handling for discovery of a unique
    # services associated with the namespace, service_type pair has been
    # hitherto handled by a call to discover_namespace
    try:
        service_name = rosservice.rosservice_find(service_type)
    except rosservice.ROSServiceIOException as e:
        print(console.red + "ERROR: {0}".format(str(e)) + console.reset)
        sys.exit(1)
    return service_name[0]

