#
# License: BSD
#   https://raw.github.com/stonier/py_trees_ros/license/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
ROS extensions, behaviours and utilities for py_trees.
"""

##############################################################################
# Imports
##############################################################################

from . import actions
from . import battery
from . import blackboard
from . import conversions
from . import mock
from . import programs
from . import subscribers
from . import trees

# Don't crucify the runtime for users if they don't have qt around
# This is a partial workaround, the qt dependencies in package.xml will still
# cause a headache. 
#
#     https://github.com/splintered-reality/py_trees_ros/issues/150
try:
    import python_qt_binding
    from . import tutorials
except ImportError:
    pass

from . import utilities
from . import visitors

##############################################################################
# Version
##############################################################################

from .version import __version__
