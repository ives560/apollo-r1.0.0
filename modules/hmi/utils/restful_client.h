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
 * @file restful_client.h
 * @brief the class of RestfulClient
 */

#ifndef MODULES_HMI_UTILS_RESTFUL_CLIENT_H_
#define MODULES_HMI_UTILS_RESTFUL_CLIENT_H_

#include <google/protobuf/message.h>
#include <string>

/**
 * @namespace apollo::hmi
 * @brief apollo::hmi
 */
namespace apollo {
namespace hmi {

/**
 * @class RestfulClient
 *
 * @brief 客户端将请求发送到restful api。 
 * RestfulClient类提供了两个公共函数，它们支持使用APT url构造客户端，并将proto消息发布到目标API。
 */
class RestfulClient {
 public:
  enum STATUS {
    SUCCESS,
    LOGIC_ERROR,
    RUNTIME_ERROR,
  };

  /*
   * @brief 构造函数，使用 explicit 关键字避免隐式构造。
   * It init client with an APT url
   * @param url the API url string.
   */
  explicit RestfulClient(const std::string& url) : url_(url) {}

  /*
   * @brief 发布一个proto消息到目标API。注意，数据是以JSON格式传输的。
   * @param proto the proto to be posted to target API.
   * @return the status define by google::protobuf::util::MessageToJsonString
   */
  STATUS Post(const google::protobuf::Message& proto);

 private:
  const std::string url_;
};

}  // namespace hmi
}  // namespace apollo

#endif  // MODULES_HMI_UTILS_RESTFUL_CLIENT_H_
