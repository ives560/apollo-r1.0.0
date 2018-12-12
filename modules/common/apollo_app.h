/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 */

#ifndef MODULES_APOLLO_APP_H_
#define MODULES_APOLLO_APP_H_

#include <csignal>
#include <string>

#include "gflags/gflags.h"
#include "modules/common/log.h"
#include "modules/common/status/status.h"
#include "modules/hmi/utils/hmi_status_helper.h"

#include "third_party/ros/include/ros/ros.h"

/**
 * @namespace apollo::common
 * @brief apollo::common
 */
namespace apollo {
namespace common {

/**
 * @class ApolloApp
 *
 * @brief 定义Apollo应用程序接口的基本模块类。
 * Apollo应用程序无限运行，直到被SIGINT或ROS关闭。
 * Apollo中的许多重要组件，例如本地化和控制，都是Apollo应用程序的例子。
 * APOLLO_MAIN宏帮助开发人员在一行中设置glog、gflag和ROS。
 */
class ApolloApp {
 public:
  /**
   * @brief module name. It is used to uniquely identify the app.
   */
  virtual std::string Name() const = 0;

  /**
   * @brief 这是Apollo应用程序的入口点。
   * 它初始化该应用程序，启动该应用程序，并在ros关闭时停止该应用程序。
   */
  virtual int Spin();

  /**
   * The default destructor.
   */
  virtual ~ApolloApp() = default;

 protected:
  /**
   * @brief 模块初始化函数。
   * 这是应用程序启动时调用的第一个函数。
   * 通常这个函数加载配置，从传感器或其他模块订阅数据。
   * @return Status initialization status
   */
  virtual apollo::common::Status Init() = 0;

  /**
   * @brief 模块启动函数。
   * Apollo app通常以两种方式触发执行: 1. 由上游消息触发，2. 由定时器触发。
   * 如果应用程序是由上游消息触发的，Start()函数通常注册一个回调函数，当接收到上游消息时将调用该函数。
   * 如果应用程序是由计时器触发的，Start()函数通常注册一个计时器回调函数。
   * @return Status start status
   */
  virtual apollo::common::Status Start() = 0;

  /**
   * @brief 模块停止函数。
   * 当ros::shutdown()函数完成后，将调用此函数。
   * 在默认的APOLLO_MAIN宏中，当接收到SIGINT时调用ros::shutdown()。
   */
  virtual void Stop() = 0;

  /**
   * @brief report module status to HMI
   * @param status HMI status
   */
  virtual void ReportModuleStatus(
      const apollo::hmi::ModuleStatus::Status status);

  apollo::hmi::ModuleStatus status_;
};

void apollo_app_sigint_handler(int signal_num);

}  // namespace common
}  // namespace apollo

#define APOLLO_MAIN(APP)                                       \
  int main(int argc, char **argv) {                            \
    google::InitGoogleLogging(argv[0]);                        \
    google::ParseCommandLineFlags(&argc, &argv, true);         \
    signal(SIGINT, apollo::common::apollo_app_sigint_handler); \
    APP apollo_app_;                                           \
    ros::init(argc, argv, apollo_app_.Name());                 \
    apollo_app_.Spin();                                        \
    return 0;                                                  \
  }

#endif  // MODULES_APOLLO_APP_H_
