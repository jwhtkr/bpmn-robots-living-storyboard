#!/usr/bin/env python
"""This module contains the ROS node class for BPMN message handling"""

from external_behavior_node import BehaviorNode
from bpmn import MessageTask

MESSAGE_SENT = "Message \"{}\" sent "

class MessageNode(BehaviorNode, MessageTask):
    """This class handles external messages and sends them to the BPMN engine"""
    def __init__(self, behavior):
        super(MessageNode, self).__init__(behavior)
        self.super = super(MessageNode, self)
        self.options["sendmessage"] = self.send_message

    def send_message(self, variables=()):
        if self.busy:
            if variables:
                self.super.send_message(self.curr_task[u"activityId"],
                                        str(self.Variables(variables,
                                                           self.encoder)))
            else:
                self.super.send_message(self.curr_task[u"activityId"])
            self.status_pub.publish(MESSAGE_SENT
                                    .format(self.curr_task[u"activityId"]))
            self.complete()


if __name__ == "__main__":
    MessageNode("Messages").run()
