#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
.. argparse::
   :module: py_trees_ros.programs.blackboard_watcher
   :func: command_line_argument_parser
   :prog: py-trees-blackboard-watcher

Example interaction with the services of a :class:`Blackboard Exchange <py_trees_ros.blackboard.Exchange>`:

.. image:: images/watcher.gif
"""

##############################################################################
# Imports
##############################################################################

import argparse
import functools
import py_trees_msgs.srv as py_trees_srvs
import rospy
import py_trees.console as console
import rosservice
import sys
import std_msgs.msg as std_msgs

##############################################################################
# Classes
##############################################################################


def description(formatted_for_sphinx):
    short = "Open up a window onto the blackboard!\n"
    examples = ["--list-variables", "access_point odom/pose/pose/position"]
    script_name = "py-trees-blackboard-watcher"

    if formatted_for_sphinx:
        # for sphinx documentation (doesn't like raw text)
        s = short
        s += "\n"
        s += "**Examples:**\n\n"
        s += ".. code-block:: bash\n"
        s += "    \n"
        s += '\n'.join(["    $ {0} {1}".format(script_name, example_args) for example_args in examples])
        s += "\n"
    else:
        banner_line = console.green + "*" * 79 + "\n" + console.reset
        s = "\n"
        s += banner_line
        s += console.bold_white + "Blackboard Watcher".center(79) + "\n" + console.reset
        s += banner_line
        s += "\n"
        s += "Open up a window onto the blackboard!\n"
        s += "\n"
        s += console.bold + "Examples" + console.reset + "\n\n"
        s += '\n'.join(["    $ " + console.cyan + script_name + console.yellow + " {0}".format(example_args) + console.reset for example_args in examples])
        s += "\n\n"
        s += banner_line
    return s


def epilog(formatted_for_sphinx):
    if formatted_for_sphinx:
        return None
    else:
        return console.cyan + "And his noodly appendage reached forth to tickle the blessed...\n" + console.reset


def command_line_argument_parser(formatted_for_sphinx=True):
    # formatted_for_sphinx is an ugly hack to make sure sphinx does not pick up the colour codes.
    # works only by assuming that the only callee who calls it without setting the arg is sphinx's argparse
    parser = argparse.ArgumentParser(description=description(formatted_for_sphinx),
                                     epilog=epilog(formatted_for_sphinx),
                                     formatter_class=argparse.RawDescriptionHelpFormatter,
                                     )
    parser.add_argument('-l', '--list-variables', action='store_true', default=None, help='list the blackboard variables')
    parser.add_argument('-n', '--namespace', nargs='?', default=None, help='namespace of blackboard services (if there should be more than one blackboard)')
    parser.add_argument('variables', nargs=argparse.REMAINDER, default=None, help='space separated list of blackboard variables to watch')
    return parser


def pretty_print_variables(variables):
    s = "\n"
    s += console.bold + console.cyan + "Blackboard Variables:" + console.reset + console.yellow + "\n"
    for variable in variables:
        variable = variable.split('/')
        if len(variable) > 1:
            sep = "/"
        else:
            sep = ""
        s += "    " * len(variable) + sep + variable[-1] + "\n"
    s += console.reset
    print "%s" % s


def echo_sub_blackboard(sub_blackboard):
    print "%s" % sub_blackboard.data


def spin_ros_node(received_topic, namespace):
    """
    Args:
        received_topic (:obj:`str`): topic name
        namespace (:obj:`str`): where to look for blackboard exchange services
    """
    rospy.init_node(received_topic.split('/')[-1])

    close_blackboard_watcher_service_name = find_service(namespace, 'py_trees_msgs/CloseBlackboardWatcher')

    def request_close_blackboard_watcher(updates_subscriber):
        """
        :param rospy.Subscriber updates_subscriber: subscriber to unregister
        """
        updates_subscriber.unregister()
        try:
            rospy.wait_for_service(close_blackboard_watcher_service_name, timeout=3.0)
            try:
                close_blackboard_watcher = rospy.ServiceProxy(close_blackboard_watcher_service_name, py_trees_srvs.CloseBlackboardWatcher)
                unused_result = close_blackboard_watcher(received_topic)  # received_topic.split('/')[-1]
                # could check if result returned success
            except rospy.ServiceException, e:
                    print(console.red + "ERROR: service call failed [%s]" % str(e) + console.reset)
                    sys.exit(1)
        except rospy.exceptions.ROSException, e:
            print(console.red + "ERROR: unknown ros exception [%s]" % str(e) + console.reset)
            sys.exit(1)

    updates_subscriber = rospy.Subscriber(received_topic, std_msgs.String, echo_sub_blackboard)
    rospy.on_shutdown(functools.partial(request_close_blackboard_watcher, updates_subscriber))
    while not rospy.is_shutdown():
        rospy.spin()


def find_service(namespace, service_type):
    try:
        service_name = rosservice.rosservice_find(service_type)
    except rosservice.ROSServiceIOException as e:
        print(console.red + "ERROR: {0}".format(str(e)) + console.reset)
        sys.exit(1)
    if len(service_name) > 0:
        if len(service_name) == 1:
            service_name = service_name[0]
        elif namespace is not None:
            for service in service_name:
                if namespace in service:
                    service_name = service
                    break
            if type(service_name) is list:
                print(console.red + "\nERROR: multiple blackboard services found %s" % service_name + console.reset)
                print(console.red + "\nERROR: but none matching the requested '%s'" % namespace + console.reset)
                sys.exit(1)
        else:
            print(console.red + "\nERROR: multiple blackboard services found %s" % service_name + console.reset)
            print(console.red + "\nERROR: select one with the --namespace argument" + console.reset)
            sys.exit(1)
    else:
        print(console.red + "ERROR: blackboard services not found" + console.reset)
        sys.exit(1)
    return service_name


def handle_args(args):
    if args.list_variables:
        list_variables_service_name = find_service(args.namespace, 'py_trees_msgs/GetBlackboardVariables')
        try:
            rospy.wait_for_service(list_variables_service_name, timeout=3.0)
            try:
                list_variables = rospy.ServiceProxy(list_variables_service_name, py_trees_srvs.GetBlackboardVariables)
                recieved_variables = list_variables()
                pretty_print_variables(recieved_variables.variables)
            except rospy.ServiceException, e:
                print(console.red + "ERROR: service call failed [%s]" % str(e) + console.reset)
                sys.exit(1)
        except rospy.exceptions.ROSException, e:
            print(console.red + "ERROR: unknown ros exception [%s]" % str(e) + console.reset)
            sys.exit(1)
    else:
        if not args.variables:
            print(console.red + "\nERROR: please provide a list of variables to watch.\n" + console.reset)
            print("%s" % description(formatted_for_sphinx=False))
            sys.exit(1)
        else:
            variables = args.variables[0:]
            variables = [variable.strip(',[]') for variable in variables]

            open_blackboard_watcher_service = find_service(args.namespace, 'py_trees_msgs/OpenBlackboardWatcher')

            try:
                rospy.wait_for_service(open_blackboard_watcher_service, timeout=3.0)
                try:
                    open_watcher = rospy.ServiceProxy(open_blackboard_watcher_service, py_trees_srvs.OpenBlackboardWatcher)
                    response = open_watcher(variables)
                except rospy.ServiceException, e:
                    print(console.red + "ERROR: service call failed [%s]" % str(e) + console.reset)
                    sys.exit(1)

                if response is not None:
                    spin_ros_node(response.topic, args.namespace)

                else:
                    print(console.red + "\nERROR: subscribing to topic failed\n" + console.reset)
            except rospy.exceptions.ROSException, e:
                print(console.red + "ERROR: unknown ros exception [%s]" % str(e) + console.reset)
                sys.exit(1)

##############################################################################
# Main
##############################################################################


def main():
    """
    Entry point for the blackboard watcher script.
    """
    command_line_args = rospy.myargv(argv=sys.argv)[1:]
    parser = command_line_argument_parser(formatted_for_sphinx=False)
    args = parser.parse_args(command_line_args)
    handle_args(args)
