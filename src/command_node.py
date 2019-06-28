#!/usr/bin/env python
"""Node that allows for publishing user input commands"""

import rospy
from std_msgs.msg import String

if __name__ == "__main__":
    # pylint: disable=invalid-name
    rospy.init_node("command_node")
    comm_pub = rospy.Publisher("commands", String, queue_size=1)

    while not rospy.is_shutdown():
        command = raw_input("Enter a command of the form "
                            + "'Behavior Command "
                            + "[Variable_Name "
                            + "Variable_Value "
                            + "Variable_Type]*': ")
        comm_pub.publish(command)
        rospy.sleep(1)
