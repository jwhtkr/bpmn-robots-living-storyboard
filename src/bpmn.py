#!/usr/bin/env python
"""This module helps to interface with the Camunda bpmn engine"""

from __future__ import print_function
import json
import httplib2


ENGINE_URL = "http://localhost:8080/engine-rest/"
JSON_HEADER = {"content-type":"application/json"}
NAME = PASSWORD = "demo"


class CamundaRESTInteraction(object):
    """A base class for interacting with the Camunda Engine through REST"""
    def __init__(self, engine_url=ENGINE_URL, encoding='utf-8'):
        self.connection = httplib2.Http()
        self.engine_url = engine_url
        self.encoding = encoding
        self.decoder = json.JSONDecoder(encoding=self.encoding)
        self.encoder = json.JSONEncoder(encoding=self.encoding)

    def certify(self, name=NAME, password=PASSWORD):
        """Adds certification with name and password"""
        self.connection.add_credentials(name, password)

        return self

    def request_get(self, request=""):
        """Makes a simple GET request"""
        # pylint: disable=unused-variable
        resp, body = self.connection.request(self.engine_url + request)
        # pylint: enable=unused-variable

        try:
            data = self.decoder.decode(body)
        except ValueError:
            return None
        else:
            return data

    def request_get_body(self, query_body, request=""):
        """Makes a GET request with body content as a dict"""
        query_list = ["=".join([key, query_body[key]]) for key in query_body]
        query = "&".join(query_list)
        query = "?" + query
        # pylint: disable=unused-variable
        resp, out_body = self.connection.request(self.engine_url
                                                 + request
                                                 + query)
        # pylint: enable=unused-variable
        try:
            data = self.decoder.decode(out_body)
        except ValueError:
            return None
        else:
            return data

    def request_post(self, body, request=""):
        """Makes a POST request with the body as a dict"""
        # pylint: disable=unused-variable
        resp, out_body = self.connection.request(self.engine_url + request,
                                                 method="POST",
                                                 body=self.encoder.encode(body),
                                                 headers={"content-type":
                                                          "application/json"})
        # pylint: enable=unused-variable
        try:
            data = self.decoder.decode(out_body)
        except ValueError:
            return None
        else:
            return data


class ExternalTask(CamundaRESTInteraction):
    """A class for working with external tasks"""
    def __init__(self, topic, worker_id="id"):
        self.super = super(ExternalTask, self)
        self.super.__init__()
        self.super.certify()
        self.topic = topic
        self.worker_id = worker_id

    def get_list(self):
        """Get the list of tasks available on the topic"""
        if self.topic == "":
            out = self.super.request_get("external-task")
        else:
            out = self.super.request_get_body({"topicName":self.topic},
                                              request="external-task")
        return out

    def get_count(self):
        """Get the number of tasks available on the topic"""
        if self.topic == "":
            out = self.super.request_get("external-task/count")
        else:
            out = self.super.request_get_body({"topicName":self.topic},
                                              request="external-task/count")
        return out

    def fetch_and_lock(self, max_tasks=1, lock_duration=1000):
        """FetchAndLock #max_tasks tasks for #lock_duration with #worker_id"""
        topics = [{"topicName":self.topic, "lockDuration":lock_duration}]
        req_body = {"workerId":self.worker_id,
                    "maxTasks":max_tasks,
                    "topics":topics}
        return self.super.request_post(req_body,
                                       request="external-task/fetchAndLock")

    def complete(self, task_id, variables=""):
        """Completes the task given by #task_id with #variables"""
        req_body = {"workerId":self.worker_id,
                    "variables":variables}
        return self.super.request_post(req_body,
                                       request="external-task/{}/complete"
                                       .format(task_id))

    def extend_lock(self, task_id, new_duration=1000):
        """Extends the lock for task #task_id for #new_duration (msec)"""
        req_body = {"workerId":self.worker_id,
                    "newDuration":new_duration}
        return self.super.request_post(req_body,
                                       request="external-task/{}/extendLock"
                                       .format(task_id))


class MessageTask(ExternalTask):
    """Extension of the External Task to send messages to the BPMN engine"""
    def __init__(self, topic, worker_id=""):
        super(MessageTask, self).__init__(topic, worker_id)
        self.super = super(MessageTask, self)

    def send_message(self, message_name, correlation_keys=""):
        """Sends a BPMN message to the BPMN engine"""
        req_body = {"messageName":message_name,
                    "correlationKeys":correlation_keys}
        return self.super.request_post(req_body, request="message")

if __name__ == "__main__":
    # pylint: disable=invalid-name
    print(str(CamundaRESTInteraction().certify().request_get("engine")))
    print(str(ExternalTask("myTopic").get_list()))
    print(str(ExternalTask("myTopic").get_count()))
    tasks = ExternalTask("myTopic").fetch_and_lock(lock_duration=10000)
    t_id = tasks[0][u"id"]
    print(str(ExternalTask("myTopic").extend_lock(t_id)))
    # pylint: enable=invalid-name
