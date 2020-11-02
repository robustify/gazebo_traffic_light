#include "GazeboTrafficLight.hpp"

namespace gazebo {

  GazeboTrafficLight::GazeboTrafficLight() {
    sequence_timestamp_ = 0.0;
  }

  void GazeboTrafficLight::Load(physics::ModelPtr model, sdf::ElementPtr sdf) {

    ROS_INFO_STREAM("Loaded traffic light plugin for model: " << model->GetName());

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

    // TODO: Get unique name somehow
    n_.reset(new ros::NodeHandle(model->GetName()));
    light_timer_ = n_->createTimer(ros::Duration(0.1), &GazeboTrafficLight::timerCb, this);
    srv_.reset(new dynamic_reconfigure::Server<gazebo_traffic_light::GazeboTrafficLightConfig>(*n_));
    srv_->setCallback(boost::bind(&GazeboTrafficLight::reconfig, this, _1, _2));

    // TODO: Create a way to set the sequence from YAML and/or service
    light_sequence_.push_back(LightSequenceEntry(LightColor::GREEN, 4.0, false));
    light_sequence_.push_back(LightSequenceEntry(LightColor::YELLOW, 2.0, false));
    light_sequence_.push_back(LightSequenceEntry(LightColor::RED, 6.0, false));
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
