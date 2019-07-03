/*
@File: external_behavior.cpp
@Date: 28 June 2019
@Author: Justin Whitaker

@Brief
This is a ROS node class that provides just enough functionality to demo the BPMN engine
*/

#include "living_storyboard/external_behavior.hpp"

#include <algorithm>

#include "ros/ros.h"

void str_to_lower(std::string& str);
std::vector<std::string> split(std::string str, const std::string delimeters=" ");

ExternalBehavior::ExternalBehavior(const std::string& behavior, const std::string& uri)
  : m_client(uri),
    m_worker_id(behavior),
    m_behavior(behavior),
    m_uri(uri),
    m_is_busy(false)
{}

void ExternalBehavior::command_cb(const std_msgs::String::ConstPtr& command_msg)
{
  std::string command_str = command_msg->data;
  std::vector<std::string> commands;

  str_to_lower(command_str);
  std::cout << command_str << std::endl;
  commands = split(command_str);
  for (auto command : commands)
  {
    std::cout << command << std::endl;
  }
  if (commands.size() > 0)
  {
    if (commands[0] == this->m_behavior)
    {
      std::cout << "The command matched the behavior!" << std::endl;
    }
  }
}

void str_to_lower(std::string& str)
{
  std::transform(str.begin(), str.end(), str.begin(), ::tolower);
}

std::vector<std::string> split(std::string str, const std::string delimeters)
{
  std::vector<std::string> result;

  char * cstr;
  size_t len;
  const char * cdelims = delimeters.c_str();
  char * ptok;

  len = str.copy(cstr, str.length());
  cstr[len] = '\0';

  ptok = std::strtok(cstr, cdelims);
  result.push_back(std::string(ptok));
  while (ptok != NULL)
  {
    ptok = strtok(NULL, cdelims);
    result.push_back(std::string(ptok));
  }

  return result;
}

const std::vector<web::json::value> poll_tasks()
{
  const std::vector<web::json::value> result;

  return result;
}

const camunda::LockResponse get_task(const camunda::LockRequest request)
{
  const camunda::LockResponse result;



  return result;
}

bool new_task(const std::vector<web::json::value> tasks)
{
  bool result = false;



  return result;
}

bool curr_task_cancelled(const std::vector<web::json::value> tasks)
{
  bool result = false;



  return result;
}

void complete(camunda::Variables variables)
{

}

void cancel(camunda::Variables variables)
{

}

void error(camunda::Variables variables)
{

}

void send_signal(camunda::Variables variables)
{

}

void send_messages(camunda::Variables variables)
{

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_node");
  ros::NodeHandle nh;
  ExternalBehavior test("behavior");
  ros::Subscriber sub = nh.subscribe<std_msgs::String>("commands", 1, &ExternalBehavior::command_cb, &test);
  ros::spin();
  return 0;
}
