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
#include "ros/console.h"

std::string str_to_lower(std::string str);
std::vector<std::string> split(std::string& str);
camunda::Variables variables_from_vector(std::vector<std::string>& variables);

ExternalBehavior::ExternalBehavior(const std::string& behavior, const std::string& uri, const uint32_t lock_duration)
  : m_client(uri),
    m_worker_id(behavior),
    m_behavior(behavior),
    m_behavior_lower(str_to_lower(behavior)),
    m_uri(uri),
    m_topics(behavior, lock_duration),
    m_is_busy(false),
    m_error_handler(uri.substr(0, 22), behavior, behavior)
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
      // std::cout << "The command matched the behavior!" << std::endl;

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
  else if (command == "error" || command == "signal" || command == "message")
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
      // std::cout << "In ExternalBehavior::do_command: " << e.what() << std::endl;
      // std::cout << "Error, Signal, and Message commands must have a 3rd field before the variables." << std::endl;
      // std::cout << "This field is an error message, or the name of the signal or message (respectively)." << std::endl;
      ROS_WARN_STREAM("In ExternalBehavior::do_command: " << e.what());
      ROS_WARN_STREAM("Error, Signal, and Message commands must have a 3rd field before the variables.");
      ROS_WARN_STREAM("This field is an error message, or the name of the signal or message (respectively).");
    }

  }
  else
  {
    ROS_WARN_STREAM("The command: " << command << " is an invalid command.");
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
      result.addVariable(variables.at(i), variables.at(i+1), variables.at(i+2));
    }
  }
  catch (std::out_of_range& e)
  {
    // std::cout << "In variables_from_vector: " << e.what() << std::endl;
    // std::cout << "Variables must be specified by [variable_name variable_value variable_type]" << std::endl;
    ROS_WARN_STREAM("In variables_from_vector: " << e.what());
    ROS_WARN_STREAM("Variables must be specified by [variable_name variable_value variable_type].");
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



const web::json::value ExternalBehavior::poll_tasks()
{
  std::stringstream ss;
  ss << "external-task?topicName=" << this->m_behavior;

  web::http::http_response response;
  response = this->m_client.request(web::http::methods::GET, ss.str()).get();

  const web::json::value result(response.extract_json().get());
  // for (size_t i = 0; i < result.size(); i++)
  // {
  //   std::cout << i << ": " << result.at(i) << std::endl;
  // }


  return result;
}

// const camunda::LockResponse ExternalBehavior::get_task(uint32_t lock_duration)
// {
//   camunda::LockRequest request(this->m_worker_id, 1);
//   camunda::Topics topics()
//   web::http::http_response response;

//   request.addTopics()
//   response = this->m_client.request(web::http::methods::POST, "external-task/fetchAndLock", request.cgetLockRequested()).get();
//   const camunda::LockResponse result;

//   return result;
// }

// bool ExternalBehavior::new_task(const std::vector<web::json::value>& tasks)
// {
//   bool result = false;



//   return result;
// }

bool ExternalBehavior::curr_task_canceled(const web::json::value curr_task_id)
{
  bool result = true;

  web::json::value tasks = this->poll_tasks();

  for (size_t i = 0; i < tasks.size(); i++)
  {
    web::json::value task = tasks[i];
    if (task["id"] == curr_task_id)
    {
      result = false;
    }
  }
  if (result)
  {
    // std::cout << "Task Cancelled: " << curr_task_id << std::endl;
    ROS_INFO_STREAM("Task canceled: " << curr_task_id);
  }


  return result;
}



void ExternalBehavior::complete(const camunda::Variables& variables)
{
  // std::cout << "Completing " << this->m_behavior << " with variables:" << std::endl;
  variables.print();
  // ROS_INFO_STREAM("Completing " << this->m_behavior << " with variables:\n" << variables.cget().serialize());

  camunda::CompleteRequest complete_request(this->m_worker_id, variables);
  complete_request.print();
  (this->m_p_curr_task)->complete(complete_request);
}

void ExternalBehavior::error(const std::string& message, const camunda::Variables& variables)
{
  // std::cout << "Sending Error with message: " << message << " and variables:" << std::endl;
  // variables.print();
  ROS_INFO_STREAM("Sending Error with message: " << message << " and variables:\n" << variables.cget().serialize());

  this->m_error_handler.addExternalTaskId(this->m_p_curr_task->getTaskId().as_string());
  this->m_error_handler.throwBpmnError("error code", message, variables);
}

void ExternalBehavior::send_signal(const std::string& signal_name, const camunda::Variables& variables)
{
  // std::cout << "Sending signal " << signal_name << " with variables:" << std::endl;
  // variables.print();
  ROS_INFO_STREAM("Sending signal " << signal_name << " with variables:\n" << variables.cget().serialize());

  camunda::ThrowSignal signal(signal_name, variables);
  this->m_client.request(web::http::methods::POST, "signal", signal.getSignal()).wait();
}

void ExternalBehavior::send_message(const std::string& message_name, const camunda::Variables& variables)
{
  // std::cout << "Sending message " << message_name << " with variables:" << std::endl;
  // variables.print();
  ROS_INFO_STREAM("Sending message " << message_name << " with variables:\n" << variables.cget().serialize());

  camunda::Json message_json;
  message_json.add("messageName", web::json::value(message_name));
  message_json.add("processVariables", variables.cget());

  this-m_client.request(web::http::methods::POST, "message", message_json.cget()).wait();
}



const std::string& ExternalBehavior::get_worker_id()
{
  return this->m_worker_id;
}

const camunda::Topics& ExternalBehavior::get_topics()
{
  if(!this->m_topics.is_safe())
  {
    std::cout << "problem" <<std::endl;
  }
  return this->m_topics;
}

const std::unique_ptr<bpmn::TaskLock<>>& ExternalBehavior::get_curr_task_ptr()
{
  return this->m_p_curr_task;
}



void ExternalBehavior::set_curr_task(bpmn::TaskLock<>* task_ptr)
{
  if (task_ptr == nullptr)
  {
    this->m_p_curr_task.reset();
  }
  else
  {
    this->m_p_curr_task.reset(task_ptr);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_node");
  ros::NodeHandle nh;
  ros::Rate rate(10);
  ExternalBehavior test("Message");
  ros::Subscriber sub = nh.subscribe<std_msgs::String>("commands", 1, &ExternalBehavior::command_cb, &test);

  web::json::value tasks;

  while (ros::ok())
  {
    tasks = test.poll_tasks();
    if (tasks.size() > 0)
    {
      test.set_curr_task(new bpmn::TaskLock<>("http://localhost:8080/", test.get_worker_id(), test.get_topics()));
      // std::cout << "Got task: " << test.get_curr_task_ptr()->getTaskId() << " with variables:" << std::endl;
      // camunda::Variables(test.get_curr_task_ptr()->getResponsVars()).print();
      ROS_INFO_STREAM("Got task: "
                      << test.get_curr_task_ptr()->getTaskId().serialize()
                      << " with variables:\n"
                      << test.get_curr_task_ptr()->getResponsVars().serialize());
      while (ros::ok() && !test.curr_task_canceled(test.get_curr_task_ptr()->getTaskId()))
      {
        ros::spinOnce();
        rate.sleep();
      }

      if (!ros::ok())
      {
        // std::cout << "Unlocking task: " << test.get_curr_task_ptr()->getTaskId() << std::endl;
        ROS_INFO_STREAM("Unlocking task: " << test.get_curr_task_ptr()->getTaskId().serialize());
        test.get_curr_task_ptr()->unlock();
        test.set_curr_task(nullptr);
      }
      else
      {
        test.set_curr_task(nullptr);
      }

    }
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
