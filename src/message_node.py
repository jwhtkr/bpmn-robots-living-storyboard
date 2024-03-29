#!/usr/bin/env python
"""This module contains the ROS node class for BPMN message handling"""

from external_behavior_node import BehaviorNode, CommandFailedError, Variables
from bpmn import MessageTask

MESSAGE_SENT = "Message '{}' sent "

class MessageNode(BehaviorNode, MessageTask):
    """This class handles external messages and sends them to the BPMN engine"""
    def __init__(self, behavior):
        super(MessageNode, self).__init__(behavior)
        self.options["sendmessage"] = self.send_message

    def send_message(self, variables=None):#()):
        if self.busy:
            if variables:
                super(MessageNode, self)\
                    .send_message(self.curr_task[u"activityId"],
                                  Variables(variables, self.encoder).variables)
            else:
                super(MessageNode, self)\
                    .send_message(self.curr_task[u"activityId"])
            self.status_pub.publish(MESSAGE_SENT
                                    .format(self.curr_task[u"activityId"]))
            self.complete()
        else:
            raise CommandFailedError("there is no message task to send")


if __name__ == "__main__":
    MessageNode(u"Message").run()
