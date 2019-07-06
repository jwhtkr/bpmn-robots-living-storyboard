/*
@File: external_behavior.cpp
@Date: 28 June 2019
@Author: Justin Whitaker

@Brief
This is a ROS node class that provides just enough functionality to demo the BPMN engine
*/

#include "living_storyboard/external_behavior.hpp"

#include <algorithm>
#include <boost/tokenizer.hpp>

#include "ros/ros.h"

std::string str_to_lower(std::string str);
std::vector<std::string> split(std::string& str);
camunda::Variables variables_from_vector(std::vector<std::string>& variables);

ExternalBehavior::ExternalBehavior(const std::string& behavior, const std::string& uri)
  : m_client(uri),
    m_worker_id(behavior),
    m_behavior(behavior),
    m_behavior_lower(str_to_lower(behavior)),
    m_uri(uri),
    m_is_busy(false)
{}

void ExternalBehavior::command_cb(const std_msgs::String::ConstPtr& command_msg)
{
  std::string command_str = command_msg->data;
  std::vector<std::string> commands;

  commands = split(command_str);
  if (commands.size() > 1)
  {
    commands[0] = str_to_lower(commands[0]);
    commands[1] = str_to_lower(commands[1]);
    if (commands[0] == this->m_behavior_lower)
    {
      std::cout << "The command matched the behavior!" << std::endl;

      if (commands.size() > 2){
        std::vector<std::string> vars(commands.begin() + 2, commands.end());
        this->do_command(commands[1], vars);
      }
      else
      {
        this->do_command(commands[1]);
      }
    }
  }
}

void ExternalBehavior::do_command(const std::string& command, std::vector<std::string>& variables)
{
  camunda::Variables vars;

  if (command == "complete")
  {
    vars = variables_from_vector(variables);
    this->complete(vars);
  }
  else
  {
    try
    {
      std::string param = variables.at(0);
      variables.erase(variables.begin());
      vars = variables_from_vector(variables);
      if (command == "error")
      {
        this->error(param, vars);
      }
      else if (command == "signal")
      {
        this->send_signal(param, vars);
      }
      else if (command == "message")
      {
        this->send_message(param, vars);
      }
    }
    catch (std::out_of_range& e)
    {
      std::cout << "In ExternalBehavior::do_command: " << e.what() << std::endl;
      std::cout << "Error, Signal, and Message commands must have a 3rd field before the variables." << std::endl;
      std::cout << "This is an error message, or the name of the signal or message respectively" << std::endl;
    }

  }
}

void ExternalBehavior::do_command(const std::string& command)
{
  std::vector<std::string> vars;
  do_command(command, vars);
}


camunda::Variables variables_from_vector(std::vector<std::string>& variables)
{
  camunda::Variables result;

  try
  {
    for (size_t i = 0; i < variables.size(); i += 3)
    {
      camunda::Variables tmp;
      tmp["value"] = web::json::value(variables.at(i+1));
      tmp["type"] = web::json::value(variables.at(i+2));
      result[variables.at(i)] = tmp.getVariables();
    }
  }
  catch (std::out_of_range& e)
  {
    std::cout << "In variables_from_vector: " << e.what() << std::endl;
    std::cout << "Variables must be specified by [variable_name variable_value variable_type]" << std::endl;
  }

  return result;
}

std::string str_to_lower(std::string str)
{
  std::transform(str.begin(), str.end(), str.begin(), ::tolower);
  return (str);
}

std::vector<std::string> split(std::string& str)
{
  std::vector<std::string> result;

  boost::tokenizer<> tok(str);

  for (boost::tokenizer<>::iterator it = tok.begin(); it != tok.end(); ++it)
  {
    result.push_back(*it);
  }

  return result;
}



const std::vector<web::json::value> ExternalBehavior::poll_tasks()
{
  const std::vector<web::json::value> result;

  web::http::http_response response;
  web::json::value response_json;

  response = this->m_client.request(web::http::methods::GET, "external-task").get();
  response_json = response.extract_json().get();


  return result;
}

const camunda::LockResponse ExternalBehavior::get_task(const camunda::LockRequest& request)
{
  const camunda::LockResponse result;



  return result;
}

bool ExternalBehavior::new_task(const std::vector<web::json::value>& tasks)
{
  bool result = false;



  return result;
}

bool ExternalBehavior::curr_task_canceled(const std::vector<web::json::value>& tasks)
{
  bool result = false;



  return result;
}



void ExternalBehavior::complete(const camunda::Variables& variables)
{
  std::cout << "Completing " << this->m_behavior << " with variables:" << std::endl;
  variables.print();
}

void ExternalBehavior::error(const std::string& message, const camunda::Variables& variables)
{
  std::cout << "Sending Error with message: " << message << " and variables:" << std::endl;
  variables.print();
}

void ExternalBehavior::send_signal(const std::string& signal_name, const camunda::Variables& variables)
{
  std::cout << "Sending signal " << signal_name << " with variables:" << std::endl;
  variables.print();
}

void ExternalBehavior::send_message(const std::string& message_name, const camunda::Variables& variables)
{
  std::cout << "Sending message " << message_name << " with variables:" << std::endl;
  variables.print();
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