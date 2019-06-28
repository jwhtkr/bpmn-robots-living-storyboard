#!/usr/bin/env python
"""This module contains the base ROS node class that acts as a behavior node"""

from __future__ import division

import argparse

import rospy
from std_msgs.msg import String

from bpmn import ExternalTask, Signal

COMPLETE = "Completed {} behavior!"
CANCEL = "{} behavior was canceled."
START = "{} behavior was started!"
COMMAND_ATTEMPT = "Attempting to perform the {} command with variables: {}."
COMMAND_FAILED = "The {} command failed because {}."
INVALID_COMMAND = "{} is an invalid command for the {} behavior."
ERROR = "{} behavior threw a BPMN error with message: {}."
SIGNAL = "{} behavior sent a {} signal."
RATE = 1


class CommandFailedError(Exception):
    """An exception to indicate that a command failed"""
    pass


class Variables(object):
    """This converts to the BPMN engine's expected variable format"""
    def __init__(self, variables, encoder):
        """Initialize with a list of name, key, type tuples

        These name/key/type tuples must be separate elements, but also adjacent
        in the list, therefore the list must have a multiple of three elements
        that it contains.
        """
        num_elem = 3
        if len(variables) % num_elem:
            raise TypeError("The Variables class cannot be instantiated with a "
                            + "number of elements in the variables list that "
                            + "is not a multiple of {}".format(num_elem))

        self.variables = {}
        self.encoder = encoder
        for i in xrange(len(variables)//num_elem):
            self.variables[variables[num_elem*i]] =\
                {"value":variables[num_elem*i + 1],
                 "type":variables[num_elem*i + 2]}

    def __str__(self):
        return self.encoder.encode(self.variables)


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
                        "cancel":self.cancel,
                        "error":self.error,
                        "sendsignal":self.send_signal}

    def command_cb(self, command_msg):
        """The callback for when a command_msg is received.

        This checks the message to see if its namespace matches (its behavior
        name), and if so it attempts to execute the command that is sent by
        using the self.options dict
        """
        command = command_msg.data.split()
        if command:
            if command[0].lower() == self.behavior.lower():
                self.status_pub.publish(COMMAND_ATTEMPT
                                        .format(command[1], command[2:]))
                try:
                    self.options[command[1].lower()](command[2:])
                except IndexError:
                    self.options[command[1].lower()]()
                except KeyError:
                    self.status_pub.publish(INVALID_COMMAND
                                            .format(command[1], command[0]))
                except CommandFailedError as ex:
                    self.status_pub.publish(COMMAND_FAILED
                                            .format(command[1], str(ex)))


    def complete(self, variables=()):
        """Completes the current task"""
        if self.busy:
            if variables:
                super(BehaviorNode, self)\
                    .complete(self.curr_task[u"id"],
                              Variables(variables, self.encoder).variables)
            else:
                super(BehaviorNode, self).complete(self.curr_task[u"id"])
            self.status_pub.publish(COMPLETE.format(self.behavior))
        else:
            raise CommandFailedError("there is no task to complete")

    def cancel(self, variables=()):
        """Cancels(Completes by default) the current task"""
        self.complete(variables)

    def error(self, variables=()):
        """Throws a BPMN error"""
        if not variables:
            raise CommandFailedError("there is no message for the error")
        message = variables[0]
        variables = variables[1:]
        if self.busy:
            if variables:
                super(BehaviorNode, self)\
                    .bpmn_error(self.curr_task[u"id"],
                                message,
                                Variables(variables, self.encoder).variables)
            else:
                super(BehaviorNode, self).bpmn_error(self.curr_task[u"id"],
                                                     message)
            self.status_pub.publish(ERROR.format(self.behavior, message))
        else:
            raise CommandFailedError("this behavior is not active and can't "
                                     + "throw an error")

    def send_signal(self, variables=()):
        """Sends a BPMN Signal with optional variables"""
        if not variables:
            raise CommandFailedError("No Signal name specified to raise")
        name = variables[0]
        variables = variables[1:]
        if self.busy:
            signal = Signal(name)
            if variables:
                signal.throw(Variables(variables, self.encoder).variables)
            else:
                signal.throw()
            self.status_pub.publish(SIGNAL.format(self.behavior, name))
        else:
            raise CommandFailedError("this behavior is not active and can't "
                                     + "send a signal")

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
                self.curr_task = self.get_task()
                if self.curr_task:
                    self.curr_task = self.curr_task[0]
                    self.busy = True
                    self.status_pub.publish(START.format(self.behavior))
            if self.curr_task_canceled(tasks):
                self.curr_task = None
                self.busy = False
                self.status_pub.publish(CANCEL.format(self.behavior))

            if self.busy:
                self.extend_lock(self.curr_task[u"id"],
                                 new_duration=(1./self.rate*1500))

            self.ros_rate.sleep()


if __name__ == "__main__":
    # pylint: disable=invalid-name
    import sys
    myargv = rospy.myargv(sys.argv)[1:]
    parser = argparse.ArgumentParser()
    parser.add_argument("topic",
                        help="The BPMN topic corresponding to this node")
    args = parser.parse_args(myargv)
    BehaviorNode(args.topic).run()
