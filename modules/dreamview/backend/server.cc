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

#include <chrono>           //标准库头文件,此头文件是日期和时间库的一部分
#include <sstream>
#include <thread>
#include "CivetServer.h"   //Embedded C/C++ web server CivetWeb
#include "gflags/gflags.h"
#include "google/protobuf/util/json_util.h"

#include "modules/common/log.h"
#include "modules/common/time/time.h"
#include "modules/dreamview/backend/simulation_world_service.h"
#include "modules/dreamview/backend/websocket.h"

DEFINE_string(static_file_dir, "modules/dreamview/frontend/dist",
              "The path to the dreamview distribution directory. The default "
              "value points to built-in version from the Apollo project."); //定义静态文件路径
DEFINE_int32(server_port, 8888, "The port of backend webserver");           //定义端口号

namespace apollo {
namespace dreamview {

using apollo::common::time::AsInt64;
using apollo::common::time::Clock;
using apollo::common::time::millis;

using Json = nlohmann::json;

/**
 * @class SimulationWorldUpdater
 * @brief 一个包装器，围绕着SimulationWorldService和WebsocketServer，
 * 通过websocket将SimulationWorld推送到前台，同时处理前台的响应。
 */
class SimulationWorldUpdater {
 public:
  /**
   * @brief Constructor with the websocket server port.
   * @param port The port on which the websocket server will be launched.
   */
  explicit SimulationWorldUpdater(int websocket_port)
      : sim_world_service_(),
        websocket_(websocket_port, WebsocketServer::NO_LOG) {
    websocket_.Run();
  }

  /**
   * @brief 回调函数从SimulationWorldService获取更新，
   * 并在触发定期计时器时通过websocket将更新推送到前端客户端。
   * @param event Timer event
   */
  void OnPushTimer(const ros::TimerEvent& event) {
    if (!sim_world_service_.ReadyToPush()) {                        //检查SimulationWorld对象是否有足够的信息。
      AWARN << "Not sending simulation world as the data is not ready!";
      return;
    }
    auto json = sim_world_service_.GetUpdateAsJson();               //返回SimulationWorld对象的json。
    websocket_.SendData(json.dump());                               //将数据发送到所有连接的客户端。
  }

 private:
  SimulationWorldService sim_world_service_;                        //仿真后台的一个主要组件，它维护一个SimulationWorld对象并不断更新它。
  WebsocketServer websocket_;                                       //websocket协议的服务器。
};

}  // namespace dreamview
}  // namespace apollo

/// Time interval, in seconds, between pushing SimulationWorld to frontend.
static constexpr double kTimeInterval = 0.1;

//后台入口
int main(int argc, char** argv) {
  using apollo::common::adapter::AdapterManager;
  using apollo::dreamview::SimulationWorldUpdater;

  ::google::InitGoogleLogging("dreamview");
  ::google::ParseCommandLineFlags(&argc, &argv, true);
  ros::init(argc, argv, "dreamview");

  // 初始化并运行服务于dreamview html和javas脚本的静态文件服务器。
  CivetServer server({"document_root", FLAGS_static_file_dir, "listening_ports",
                      std::to_string(FLAGS_server_port)});

  // Websocket port number is web server port number + 1.
  SimulationWorldUpdater updater(FLAGS_server_port + 1);

  //创建定时器,将SimulationWorld推送到前端
  auto timer = AdapterManager::CreateTimer(ros::Duration(kTimeInterval),
                                           &SimulationWorldUpdater::OnPushTimer,
                                           &updater);

  ros::spin();
  return 0;
}
