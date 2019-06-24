#!/usr/bin/env python
"""This module helps to interface with the Camunda bpmn engine"""

import json
import httplib2


ENGINE_URL = "http://localhost:8080/engine-rest/"
JSON_HEADER = {"content-type":"application/json"}
NAME = PASSWORD = "demo"
# PASSWORD = "demo"


class CamundaRESTInteraction(object):
    """A base class for interacting with the Camunda Engine through REST"""
    def __init__(self, engine_url=ENGINE_URL, encoding='utf-8'):
        self.connection = httplib2.Http()
        self.engine_url = engine_url
        self.encoding = encoding
        self.decoder = json.JSONDecoder(encoding=self.encoding)
        self.encoder = json.JSONEncoder(encoding=self.encoding)

    def certify(self, name=NAME, password=PASSWORD):
        """Certifies with name and password"""
        self.connection.add_credentials(name, password)

        return self

    def get_request(self, request=""):
        """Makes a simple GET request as it was previously built"""
        # pylint: disable=unused-variable
        resp, body = self.connection.request(self.engine_url + request)
        # pylint: enable=unused-variable

        data = self.decoder.decode(body)

        return data

    def get_request_body(self, body, request=""):
        """Makes a get request with body content as a dict"""
        # pylint: disable=unused-variable
        resp, out_body = self.connection.request(self.engine_url + request,
                                                 body=self.encoder.encode(body))
        # pylint: enable=unused-variable
        data = self.decoder.decode(out_body)
        return data


class ExternalTask(CamundaRESTInteraction):
    """A class for working with external tasks"""
    def __init__(self, topic=""):
        self.super = super(ExternalTask, self)
        self.super.__init__()
        self.super.certify()
        self.topic = topic

    def get_list(self):
        """Get the list of tasks available on the topic"""
        if self.topic == "":
            out = self.super.get_request("external-task")
        else:
            out = self.super.get_request_body({"topicName":self.topic}, "external-task")
        return out

    def get_count(self):
        """Get the number of tasks available on the topic"""
        super(ExternalTask, self).get_request("external-task/count")

if __name__ == "__main__":
    print str(CamundaRESTInteraction().certify().get_request("engine"))
    # print str(CamundaRESTInteraction().certify().get_request_query({"topicName":"myTopic", "notLocked":"true", "withRetriesLeft":"true", "active":"true"}, request="external-task"))
    print str(ExternalTask("myTopic").get_list())
