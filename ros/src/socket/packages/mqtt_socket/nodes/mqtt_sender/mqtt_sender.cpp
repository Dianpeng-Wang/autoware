/*
 *  Copyright (c) 2017, Tier IV, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "stdlib.h"
#include "string.h"
#include "MQTTClient.h"
#include "mqtt_socket/mqtt_setting.hpp"

class MqttSender
{
public:
  MqttSender();
  ~MqttSender();
  // static void connlost(void *context, char *cause);
  static void canInfoCallback(const std_msgs::String::ConstPtr& msg);

private:
  static ros::Subscriber vehicle_info_sub_;
	ros::NodeHandle node_handle_;
  static MQTTClient_message pubmsg_;
  static MQTTClient_deliveryToken deliveredtoken_;

  static MQTTClient mqtt_client_;
  static std::string mqtt_address_;
  static std::string mqtt_topic_;
  static std::string mqtt_client_id_;
  static int mqtt_qos_;
  static int mqtt_timeout_;
};

ros::Subscriber MqttSender::vehicle_info_sub_;
MQTTClient_message MqttSender::pubmsg_;
MQTTClient_deliveryToken MqttSender::deliveredtoken_;
MQTTClient MqttSender::mqtt_client_;
std::string MqttSender::mqtt_address_;
std::string MqttSender::mqtt_topic_;
std::string MqttSender::mqtt_client_id_;
int MqttSender::mqtt_qos_;
int MqttSender::mqtt_timeout_;

MqttSender::MqttSender() :
    node_handle_("~")
{
  // ROS Publisher
  vehicle_info_sub_ = node_handle_.subscribe("/mqtt_receiver/remote_vehicle_cmd", 1000, canInfoCallback);

  // MQTT PARAMS
  pubmsg_ = MQTTClient_message_initializer;
  mqtt_address_ = ADDRESS;
  mqtt_topic_ = std::string(SENDER_TOPIC) + std::string(VEHICLEID) + "/caninfo";
  mqtt_client_id_ = std::string(CLIENTID) + "_" + std::string(VEHICLEID) + "_snd";
  mqtt_qos_ = QOS;
  mqtt_timeout_ = TIMEOUT;

  node_handle_.param("/confing/mqtt/address", mqtt_address_, mqtt_address_);
  node_handle_.param("/confing/mqtt/topic", mqtt_topic_, mqtt_topic_);
  node_handle_.param("/confing/mqtt/client_id", mqtt_client_id_, mqtt_client_id_);
  node_handle_.param("/confing/mqtt/qos", mqtt_qos_, mqtt_qos_);
  node_handle_.param("/confing/mqtt/timeout", mqtt_timeout_, mqtt_timeout_);
  ROS_INFO("%s, %s, %s\n", mqtt_address_.c_str(), mqtt_topic_.c_str(), mqtt_client_id_.c_str());
  MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
  int rc;

  MQTTClient_create(&mqtt_client_, mqtt_address_.c_str(), mqtt_client_id_.c_str(),
      MQTTCLIENT_PERSISTENCE_NONE, NULL);
  conn_opts.keepAliveInterval = 20;
  conn_opts.cleansession = 1;

  if ((rc = MQTTClient_connect(mqtt_client_, &conn_opts)) != MQTTCLIENT_SUCCESS)
  {
      ROS_INFO("Failed to connect, return code %d\n", rc);
      exit(EXIT_FAILURE);
  }
}

MqttSender::~MqttSender()
{
  MQTTClient_disconnect(mqtt_client_, 10000);
  MQTTClient_destroy(&mqtt_client_);
}

static void MqttSender::canInfoCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());

  pubmsg_.payload = msg->data.c_str();
  pubmsg_.payloadlen = strlen(msg->data.c_str());
  pubmsg_.qos = mqtt_qos_;
  pubmsg_.retained = 0;
  MQTTClient_publishMessage(mqtt_client_, mqtt_topic_.c_str(), &pubmsg_, &deliveredtoken_);
  printf("Waiting for up to %d seconds for publication of %s\n"
          "on topic %s for client with ClientID: %s\n",
          (int)(mqtt_timeout_/1000), msg->data.c_str(), mqtt_topic_.c_str(), mqtt_client_id_);
  int rc = MQTTClient_waitForCompletion(mqtt_client_, deliveredtoken_, mqtt_timeout_);
  printf("Message with delivery token %d delivered\n", deliveredtoken_);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mqtt_sender");
  MqttSender node;
  ros::spin();

  return 0;
}
