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

#ifndef MODULES_DREAMVIEW_BACKEND_WEBSOCKET_H_
#define MODULES_DREAMVIEW_BACKEND_WEBSOCKET_H_

#include <unordered_map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>

#include "third_party/json/json.hpp"
#include "websocketpp/client.hpp"                   // C++ websocket client/server library http://www.zaphoyd.com/websocketpp
#include "websocketpp/config/asio_no_tls.hpp"
#include "websocketpp/config/asio_no_tls_client.hpp"
#include "websocketpp/server.hpp"

/**
 * @namespace apollo::dreamview
 * @brief apollo::dreamview
 */
namespace apollo {
namespace dreamview {

/**
 * @class WebsocketServer
 *
 * @brief 构建在websocketpp库之上的WebsocketServer表示遵循websocket协议的服务器端点。
 * 启动时，它将启动一个接受器来接收传入的连接，并在运行时管理连接池。
 * 每个连接都在服务器和客户机端点之间。
 * SendData()方法用于将数据推送到所有连接的客户机。
 */
class WebsocketServer {
 public:
  using ConnectionHandle = websocketpp::connection_hdl;
  using ConnectionSet =
      std::set<ConnectionHandle, std::owner_less<ConnectionHandle>>;
  using ServerEndpoint = websocketpp::server<websocketpp::config::asio>;
  using MessagePtr = websocketpp::config::asio_client::message_type::ptr;
  using Json = nlohmann::json;
  using MessageHandler = std::function<void(const Json &)>;

  /// With the DEFAULT_LOG_LEVEL setting, websocketpp library logs the event
  /// when a client gets connected, disconnected, or pushed with data payload.
  static constexpr websocketpp::log::level DEFAULT_LOG_LEVEL =
      (websocketpp::log::alevel::connect |
       websocketpp::log::alevel::disconnect |
       websocketpp::log::alevel::message_payload);

  /// With the NO_LOG configuration, nothing will be logged by the websocketpp
  /// library.
  static constexpr websocketpp::log::level NO_LOG =
      websocketpp::log::alevel::none;

  /**
   * @brief Constructor in which you can specify the log level.
   * @param port The port on which the server will be launched.
   * @param log_level The log level for which the events will be logged by the
   * websocketpp library, could use DEFAULT_LOG_LEVEL or NO_LOG defined above.
   */
  WebsocketServer(int port, websocketpp::log::level log_level);

  /**
   * @brief Constructor with the default log level.
   * @param port The port on which the server will be launched.
   */
  explicit WebsocketServer(int port)
      : WebsocketServer(port, DEFAULT_LOG_LEVEL) {}

  ~WebsocketServer();

  /**
   * @brief 调用此方法以实际运行服务器。该方法将为服务器侦听器和事件处理程序创建一个后台线程。
   *
   * NOTE: This method does not block.
   */
  void Run();

  /**
   * @brief 此方法将停止侦听新连接，关闭所有当前连接，并最终停止服务器的后台线程。
   */
  void Stop();

  /**
   * @brief 将提供的数据发送到所有连接的客户端。
   * @param data The message string to be sent.
   */
  bool SendData(const std::string &data);

  /**
   * @brief 为消息类型添加新的消息处理程序。
   * @param type The name/key to identify the message type.
   * @param handler The function to handle the received message.
   */
  void RegisterMessageHandler(std::string type, MessageHandler handler) {
    message_handlers_[type] = handler;
  }

 private:
  // The server endpoint.
  ServerEndpoint server_;

  // The thread for the server (listeners and handlers).
  std::unique_ptr<std::thread> server_thread_;

  // The pool of all maintained connections.
  ConnectionSet connections_;

  // Message handlers keyed by message type.
  std::unordered_map<std::string, MessageHandler> message_handlers_;

  // The mutex guarding the connection set. We are not using read
  // write lock, as the server is not expected to get many clients
  // (connections).
  mutable std::mutex mutex_;

  // OnAcceptConnection() will be called upon connection request
  // (handshake) from a new client. It returns true if the connection
  // is accepted, or false if it is rejected.
  bool OnAcceptConnection(ConnectionHandle handle);

  // OnFail() will be called when a websocket connection attempt by
  // client fails.
  void OnFail(ConnectionHandle handle);

  // OnClose() will be called when a client gets disconnected, either
  // proactively or passively.
  void OnClose(ConnectionHandle handle);

  // 当服务器从客户机接收到消息时将调用OnMessage，它将根据消息类型将消息分派给相应的处理程序。
  void OnMessage(ConnectionHandle handle, MessagePtr message);
};

}  // namespace dreamview
}  // namespace apollo

#endif /* MODULES_DREAMVIEW_BACKEND_WEBSOCKET_H_ */
