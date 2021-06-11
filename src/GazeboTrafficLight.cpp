#include "GazeboTrafficLight.hpp"

namespace gazebo {

  GazeboTrafficLight::GazeboTrafficLight() {
    sequence_timestamp_ = 0.0;
  }

  void GazeboTrafficLight::Load(physics::ModelPtr model, sdf::ElementPtr sdf) {

    ROS_INFO_STREAM("Loaded traffic light plugin for model: " << model->GetName());

    std::vector<geometry_msgs::TransformStamped> traffic_light_transforms;
    for (auto model_link : model->GetLinks()) {
      if (model_link->GetName().find("light_fixture") != std::string::npos) {
        geometry_msgs::TransformStamped new_light_transform;
        new_light_transform.header.frame_id = "world";
        new_light_transform.child_frame_id = parseLinkName(model_link->GetName());
        new_light_transform.transform.translation.x = model_link->WorldPose().Pos().X();
        new_light_transform.transform.translation.y = model_link->WorldPose().Pos().Y();
        new_light_transform.transform.translation.z = model_link->WorldPose().Pos().Z();
        new_light_transform.transform.rotation.w = model_link->WorldPose().Rot().W();
        new_light_transform.transform.rotation.x = model_link->WorldPose().Rot().X();
        new_light_transform.transform.rotation.y = model_link->WorldPose().Rot().Y();
        new_light_transform.transform.rotation.z = model_link->WorldPose().Rot().Z();
        traffic_light_transforms.push_back(new_light_transform);
      }
    }
    if (traffic_light_transforms.size() > 0) {
      broadcaster_.sendTransform(traffic_light_transforms);
    }

    for (auto model_joint : model->GetJoints()) {
      if (model_joint->GetName().find("switch") == std::string::npos) {
        continue;
      }
      std::string traffic_light_name;
      std::string joint_name;
      parseJointName(model_joint->GetName(), traffic_light_name, joint_name);
      if (joint_name.find("red") != std::string::npos) {
        traffic_lights_[traffic_light_name][RED] = model_joint;
      } else if (joint_name.find("yellow") != std::string::npos) {
        traffic_lights_[traffic_light_name][YELLOW] = model_joint;
      } else if (joint_name.find("green") != std::string::npos) {
        traffic_lights_[traffic_light_name][GREEN] = model_joint;
      }
    }

    n_.reset(new ros::NodeHandle(model->GetName()));
    light_timer_ = n_->createTimer(ros::Duration(0.1), &GazeboTrafficLight::timerCb, this);
    srv_.reset(new dynamic_reconfigure::Server<gazebo_traffic_light::GazeboTrafficLightConfig>(*n_));
    srv_->setCallback(boost::bind(&GazeboTrafficLight::reconfig, this, _1, _2));

    XmlRpc::XmlRpcValue sequence_list;
    if (n_->getParam("light_sequence", sequence_list)) {
      for (int i = 0; i < sequence_list.size(); i++) {
        XmlRpc::XmlRpcValue param_dict_list = sequence_list[i];
        LightSequenceEntry new_entry;
        bool parsable = true;

        if (param_dict_list["color"].getType() != XmlRpc::XmlRpcValue::TypeInvalid) {
          if (param_dict_list["color"].getType() == XmlRpc::XmlRpcValue::TypeString) {
            std::string color = param_dict_list["color"];
            if (color == "green") {
              new_entry.color = LightColor::GREEN;
            } else if (color == "yellow") {
              new_entry.color = LightColor::YELLOW;
            } else { // red
              new_entry.color = LightColor::RED;
            }
          } else {
            parsable = false;
            ROS_ERROR("Color attribute is not a string!");
          }
        }

        if (param_dict_list["duration"].getType() != XmlRpc::XmlRpcValue::TypeInvalid) {
          if (param_dict_list["duration"].getType() == XmlRpc::XmlRpcValue::TypeInt) {
            int temp_val = param_dict_list["duration"];
            new_entry.duration = (double)temp_val;
          } else if (param_dict_list["duration"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            new_entry.duration = param_dict_list["duration"];
          } else {
            parsable = false;
            ROS_ERROR("Duration attribute is not a numeric type!");
          }
        }

        if (param_dict_list["flashing"].getType() != XmlRpc::XmlRpcValue::TypeInvalid) {
          if (param_dict_list["flashing"].getType() == XmlRpc::XmlRpcValue::TypeBoolean) {
            new_entry.flashing = param_dict_list["flashing"];
          } else {
            parsable = false;
            ROS_ERROR("Flashing attribute is not a boolean type!");
          }
        }

        if (parsable) {
          light_sequence_.push_back(new_entry);
        }
      }
    } else {
      ROS_INFO_STREAM("light_sequence parameter not found for model " << model->GetName() << "... Using default");
      light_sequence_.push_back(LightSequenceEntry(LightColor::GREEN, 4.0, false));
      light_sequence_.push_back(LightSequenceEntry(LightColor::YELLOW, 2.0, false));
      light_sequence_.push_back(LightSequenceEntry(LightColor::RED, 6.0, false));
    }
    computeCycleTime();
  }

  void GazeboTrafficLight::parseJointName(const std::string& input_str, std::string& traffic_light_name, std::string& joint_name) {
    std::string s = input_str;
    size_t pos = s.rfind("::");
    if (pos == std::string::npos) {
      traffic_light_name = "gazebo_traffic_light";
      joint_name = s;
    } else {
      traffic_light_name = s.substr(0, pos);
      joint_name = s.substr(pos + 2);
    }
  }

  std::string GazeboTrafficLight::parseLinkName(const std::string& input_str) {
    std::string output;
    std::string s = input_str;
    int count = std::count(input_str.begin(), input_str.end(), ':') / 2;
    if (count == 0) {
      output = s;
    } else {
      int limit = std::max(1, count - 1);
      for (int i = 0; i < limit; i++) {
        size_t pos = s.find("::");
        output += (s.substr(0, pos) + "_");
        s = s.substr(pos + 2);
      }
      output = output.substr(0, output.length() - 1); // Remove trailing underscore
    }
    return output;
  }

  void GazeboTrafficLight::computeCycleTime() {
    cycle_time_ = 0.0;
    for (auto& stage : light_sequence_) {
      cycle_time_ += stage.duration;
    }
  }

  void GazeboTrafficLight::timerCb(const ros::TimerEvent& event) {
    for (auto light : traffic_lights_) {
      switch (cfg_.override) {
        case gazebo_traffic_light::GazeboTrafficLight_NO_FORCE:
        advanceSequence(light.first);
        break;
        case gazebo_traffic_light::GazeboTrafficLight_RED:
        setColor(light.first, LightColor::RED, false);
        break;
        case gazebo_traffic_light::GazeboTrafficLight_RED_FLASH:
        setColor(light.first, LightColor::RED, true);
        break;
        case gazebo_traffic_light::GazeboTrafficLight_YELLOW:
        setColor(light.first, LightColor::YELLOW, false);
        break;
        case gazebo_traffic_light::GazeboTrafficLight_YELLOW_FLASH:
        setColor(light.first, LightColor::YELLOW, true);
        break;
        case gazebo_traffic_light::GazeboTrafficLight_GREEN:
        setColor(light.first, LightColor::GREEN, false);
        break;
        case gazebo_traffic_light::GazeboTrafficLight_GREEN_FLASH:
        setColor(light.first, LightColor::GREEN, true);
        break;
      }
    }

    if (event.last_expected == ros::Time(0)) {
      return;
    }

    sequence_timestamp_ += (event.current_real - event.last_real).toSec();
    if (sequence_timestamp_ >= cycle_time_) {
      sequence_timestamp_ -= cycle_time_;
    }
  }

  void GazeboTrafficLight::advanceSequence(const std::string& traffic_light_name) {
    double time_thres = cycle_time_;
    for (auto it = light_sequence_.rbegin(); it != light_sequence_.rend(); ++it) {
      time_thres -= it->duration;
      if (sequence_timestamp_ >= time_thres) {
        setColor(traffic_light_name, it->color, it->flashing);
        break;
      }
    }
  }

  void GazeboTrafficLight::setColor(const std::string& traffic_light_name, LightColor color, bool flashing) {
    for (int i = LightColor::RED; i <= LightColor::GREEN; i++) {
      if ( (flashing && ( (int)(1.25 * sequence_timestamp_) % 2 )) || (i != color) ){
        traffic_lights_[traffic_light_name][i]->SetPosition(0, LIGHT_OFF_POS_);
      } else {
        traffic_lights_[traffic_light_name][i]->SetPosition(0, LIGHT_ON_POS_);
      }
    }
  }

  void GazeboTrafficLight::reconfig(gazebo_traffic_light::GazeboTrafficLightConfig& config, uint32_t level) {
    sequence_timestamp_ = 0.0;
    cfg_ = config;
  }

  void GazeboTrafficLight::Reset() {

  }

  GazeboTrafficLight::~GazeboTrafficLight() {
    n_->shutdown();
  }

} // namespace gazebo
