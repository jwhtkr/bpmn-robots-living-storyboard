#!../venv/bin/python
"""This module contains the base ROS node class that acts as a behavior node"""

from __future__ import division

import rospy
from std_msgs.msg import String
from bpmn import ExternalTask

COMPLETE = "Completed {} Behavior!"
CANCEL = "{} Behavior was canceled."
START = "{} Behavior was started!"
INVALID_COMMAND = "{} is an invalid command for the {} behavior."
RATE = 1

class BehaviorNode(ExternalTask):
    """This is the base behavior node class"""
    def __init__(self, behavior):
        super(BehaviorNode, self).__init__(topic=behavior, worker_id=behavior)

        rospy.init_node(behavior)
        self.rate = RATE
        self.ros_rate = rospy.Rate(self.rate)
        self.status_pub = rospy.Publisher("status", String, queue_size=10)
        rospy.Subscriber("commands", String, self.command_cb)

        self.curr_task = None
        self.busy = False
        self.behavior = behavior
        self.options = {"complete":self.complete,
                        "cancel":self.cancel}


    class Variables(object):
        """This converts to the BPMN engine's expected variable format"""
        def __init__(self, variables, encoder):
            """Initialize with a list of variable_name, variable_key pairs

            These name/key pairs must be separate elements, but also adjacent in
            the list, therefore the list must be even in the number of elements
            it contains. For now this class treats all variables as type string.
            """
            if len(variables) % 2:
                raise TypeError("The Variables class cannot be instantiated "
                                + "with an odd number of elements in the "
                                + "variables list")

            self.variables = {}
            self.encoder = encoder
            for i in xrange(len(variables)//2):
                self.variables[variables[2*i]] = {"value":variables[2*i + 1],
                                                  "type":"String"}

        def __str__(self):
            return self.encoder.encode([self.variables])

    def command_cb(self, command_msg):
        """The callback for when a command_msg is received.

        This checks the message to see if its namespace matches (its behavior
        name), and if so it attempts to execute the command that is sent by
        using the self.options dict
        """
        command = command_msg.data.split()
        if command[0].lower() == self.behavior.lower():
            try:
                self.options[command[1].lower()](command[2:])
            except IndexError:
                self.options[command[1].lower()]()
            except KeyError:
                self.status_pub.publish(INVALID_COMMAND
                                        .format(command[1], command[0]))

    def complete(self, variables=()):
        """Completes the current task"""
        if self.busy:
            if variables:
                super(BehaviorNode, self)\
                    .complete(self.curr_task[u"id"],
                              str(self.Variables(variables, self.encoder)))
            else:
                super(BehaviorNode, self).complete(self.curr_task[u"id"])
            self.status_pub.publish(COMPLETE.format(self.behavior))
            self.curr_task = None
            self.busy = False

    def cancel(self, variables=()):
        """Cancels(Completes by default) the current task"""
        self.complete(variables)

    def poll_tasks(self):
        """Gets the list of tasks for this behavior"""
        return super(BehaviorNode, self).get_list()

    def new_task(self, tasks):
        """Checks if there is a new/not serviced task"""
        # If curr_task is null and tasks is not empty,
        # or if tasks is of length 2 or greater
        # then there is a new task
        return bool((not self.curr_task and tasks)
                    or (tasks and len(tasks) > 1))

    def get_task(self):
        """Gets a task to complete"""
        return super(BehaviorNode, self)\
                   .fetch_and_lock(lock_duration=(1./self.rate * 1500.))

    def curr_task_canceled(self, tasks):
        """Checks if the task is cancelled"""
        out = False
        if not self.curr_task:
            # if curr_task is null do nothing (in order to return False)
            pass
        elif self.curr_task and not tasks:
            # if curr_task is not null and tasks is empty
            out = True
        elif self.curr_task and tasks:
            # if curr_task is not null and tasks is not empty
            for task in tasks:
                if task[u"id"] == self.curr_task[u"id"]:
                    break
            else:
                out = True
        return out

    def run(self):
        """The main node logic"""
        while not rospy.is_shutdown():
            tasks = self.poll_tasks()

            if self.new_task(tasks) and not self.busy:
                self.curr_task = self.get_task()[0]
                self.busy = True
                self.status_pub.publish(START.format(self.behavior))
            elif self.curr_task_canceled(tasks):
                self.curr_task = None
                self.busy = False
                self.status_pub.publish(CANCEL.format(self.behavior))

            if self.busy:
                self.extend_lock(self.curr_task[u"id"],
                                 new_duration=(1./self.rate*1500))

            self.ros_rate.sleep()



if __name__ == "__main__":
    BehaviorNode(u"Evacuate").run()
