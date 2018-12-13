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

#ifndef MODULES_DREAMVIEW_BACKEND_SIM_WORLD_H_
#define MODULES_DREAMVIEW_BACKEND_SIM_WORLD_H_

#include <functional>
#include <string>
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/dreamview/proto/simulation_world.pb.h"
#include "third_party/json/json.hpp"

/**
 * @namespace apollo::dreamview::internal
 * @brief apollo::dreamview::internal
 */
namespace apollo {
namespace dreamview {

namespace internal {

//模板UpdateSimulationWorld定义
template <typename AdapterType>
void UpdateSimulationWorld(const typename AdapterType::DataType &data,
                           SimulationWorld *world);

template <>
void UpdateSimulationWorld<apollo::common::adapter::MonitorAdapter>(
    const apollo::common::monitor::MonitorMessage &monitor_msg,
    SimulationWorld *world);

template <>
void UpdateSimulationWorld<apollo::common::adapter::LocalizationAdapter>(
    const apollo::localization::LocalizationEstimate &localization,
    SimulationWorld *world);

template <>
void UpdateSimulationWorld<apollo::common::adapter::ChassisAdapter>(
    const apollo::canbus::Chassis &chassis, SimulationWorld *world);

template <>
void UpdateSimulationWorld<apollo::common::adapter::PlanningTrajectoryAdapter>(
    const apollo::planning::ADCTrajectory &trajectory, SimulationWorld *world);

}  // namespace internal

/**
 * @class SimulationWorldService
 * @brief 这是仿真后台的一个主要组件，它维护一个SimulationWorld对象并不断更新它。
 * SimulationWorld表示仿真世界中所有对象的最新信息，包括汽车、规划轨迹等。
 * NOTE: This class is not thread-safe.
 */
class SimulationWorldService {
 public:
  using Json = nlohmann::json;

  // The maximum number of monitor message items to be kept in
  // SimulationWorld.
  static constexpr int kMaxMonitorItems = 30;

  /**
   * @brief Default constructor.
   */
  SimulationWorldService();

  /**
   * @brief Get a read-only view of the SimulationWorld.
   * @return Constant reference to the SimulationWorld object.
   */
  inline const SimulationWorld &world() const {
    return world_;
  }

  /**
   * @brief Returns the json representation of the SimulationWorld object.
   * @return Json object equivalence of the SimulationWorld object.
   */
  Json GetUpdateAsJson() const;

  /**
   * @brief The function Update() is periodically called to check for updates
   * from the adapters. All the updates will be written to the SimulationWorld
   * object to reflect the latest status.
   * @return Constant reference to the SimulationWorld object.
   */
  const SimulationWorld &Update();

  /**
   * @brief 检查SimulationWorld对象是否有足够的信息。
   * 如果没有准备好，后端不会将SimulationWorld推送到前端。
   * @return True if the object is ready to push.
   */
  bool ReadyToPush() const {
    return world_.has_auto_driving_car();       //auto_driving_car字段是否有数据
  }

 private:
  /**
   * @brief 在adatper管理器上注册一个回调函数，以便在接收新消息时更新SimulationWorld对象。
   * 由于我们使用的是单线程ROS旋转器，所以没有锁来保护它。
   */
  template <typename AdapterType>
  void RegisterDataCallback(const std::string &adapter_name,
                            AdapterType *adapter) {
    if (adapter == nullptr) {
      AFATAL << adapter_name << " adapter is not correctly initialized. "
                                "Please check the adapter manager ";
    }

    //Set##name##Callback
    adapter->SetCallback(
        std::bind(&internal::UpdateSimulationWorld<AdapterType>,
                  std::placeholders::_1, &world_));
  }

  // The underlying SimulationWorld object, owned by the
  // SimulationWorldService instance.
  SimulationWorld world_;           //SimulationWorld 消息，由simulation_world.proto文件生成
};

}  // namespace dreamview
}  // namespace apollo

#endif /* MODULES_DREAMVIEW_BACKEND_SIM_WORLD_H_ */
