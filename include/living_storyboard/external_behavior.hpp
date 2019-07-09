/*
@File: ExternalBehavior.hpp
@Date: 28 June 2019
@Author: Justin Whitaker

@Brief
This is a ROS node class that provides just enough functionality to demo the BPMN engine
*/

#ifndef EXTERNAL_BEHAVIOR_HPP
#define EXTERNAL_BEHAVIOR_HPP

#include "camunda_objects/lock_request.hpp"
#include "camunda_objects/complete_request.hpp"
#include "camunda_objects/json.hpp"
#include "camunda_objects/lock_response.hpp"
#include "camunda_objects/throw_signal.hpp"
#include "camunda_objects/topics.hpp"
#include "camunda_objects/variables.hpp"
#include "camunda_error_handling/bpmn_error.hpp"

#include "task_lock/task_lock.hpp"

#include "std_msgs/String.h"

#include <cpprest/json.h>
#include <cpprest/http_client.h>

#include <string>
#include <functional>
#include <map>
#include <vector>

class ExternalBehavior
{
private:
    web::http::client::http_client m_client;
    const std::string m_worker_id;
    const std::string m_behavior;
    const std::string m_behavior_lower;
    const std::string m_uri;
    const camunda::Topics m_topics;
    // const std::unordered_map<std::string, std::function<void(ExternalBehavior*, std::vector<std::string>)>> m_options;
    bool m_is_busy;
    std::unique_ptr<bpmn::TaskLock<>> m_p_curr_task;
    // web::json::value m_curr_task;


public:
    ExternalBehavior() = delete;
    ExternalBehavior(const ExternalBehavior&) = delete;
    ExternalBehavior(const std::string& behavior,
                     const std::string& uri="http://localhost:8080/engine-rest/",
                     const uint32_t lock_duration=1000);

    ~ExternalBehavior() = default;

    void command_cb(const std_msgs::String::ConstPtr& command_msg);
    void do_command(const std::string& command, std::vector<std::string>& variables);
    void do_command(const std::string& command);

    const web::json::value poll_tasks();
    // const camunda::LockResponse get_task(uint32_t lock_duration=1000);
    // bool new_task(const std::vector<web::json::value>& tasks);
    bool curr_task_canceled(const web::json::value curr_task_id);

    void complete(const camunda::Variables& variables);
    void error(const std::string& message, const camunda::Variables& variables);
    void send_signal(const std::string& signal_name, const camunda::Variables& variables);
    void send_message(const std::string& message_name, const camunda::Variables& variables);

    const std::string& get_worker_id();
    const camunda::Topics& get_topics();
    const std::unique_ptr<bpmn::TaskLock<>>& get_curr_task_ptr();


    void set_curr_task(bpmn::TaskLock<>* task_ptr);
};


#endif
