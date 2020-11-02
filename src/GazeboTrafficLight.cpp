#include "GazeboTrafficLight.hpp"

namespace gazebo {

  GazeboTrafficLight::GazeboTrafficLight() {
    sequence_timestamp_ = 0.0;
  }

  void GazeboTrafficLight::Load(physics::ModelPtr model, sdf::ElementPtr sdf) {

    update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboTrafficLight::OnUpdate, this, _1));
    
    for (auto model_joint : model->GetJoints()) {
      if (model_joint->GetName().find("switch") == std::string::npos) {
        continue;
      }

      if (model_joint->GetName().find("green") != std::string::npos) {
        green_switch_joint_ = model_joint;
      } else if (model_joint->GetName().find("yellow") != std::string::npos) {
        yellow_switch_joint_ = model_joint;
      } else if (model_joint->GetName().find("red") != std::string::npos) {
        red_switch_joint_ = model_joint;
      }
    }

    if (!green_switch_joint_) {
      ROS_ERROR("Green joint not found!");
    }
    if (!yellow_switch_joint_) {
      ROS_ERROR("Yellow joint not found!");
    }
    if (!red_switch_joint_) {
      ROS_ERROR("Red joint not found!");
    }

    // TODO: Get unique name somehow
    n_.reset(new ros::NodeHandle());
    srv_.reset(new dynamic_reconfigure::Server<gazebo_traffic_light::GazeboTrafficLightConfig>(ros::NodeHandle(*n_, "traffic_light")));
    srv_->setCallback(boost::bind(&GazeboTrafficLight::reconfig, this, _1, _2));

    // TODO: Create a way to set the sequence from YAML and/or service
    light_sequence_.push_back(LightSequenceEntry(LightColor::GREEN, 4.0, false));
    light_sequence_.push_back(LightSequenceEntry(LightColor::YELLOW, 2.0, false));
    light_sequence_.push_back(LightSequenceEntry(LightColor::RED, 6.0, false));
    computeCycleTime();
  }

  void GazeboTrafficLight::computeCycleTime() {
    cycle_time_ = 0.0;
    for (auto& stage : light_sequence_) {
      cycle_time_ += stage.duration;
    }
  }

  void GazeboTrafficLight::OnUpdate(const common::UpdateInfo& info) {
    switch (cfg_.override) {
      case gazebo_traffic_light::GazeboTrafficLight_NO_FORCE:
      advanceSequence();
      break;
      case gazebo_traffic_light::GazeboTrafficLight_RED:
      setColor(LightColor::RED, false);
      break;
      case gazebo_traffic_light::GazeboTrafficLight_RED_FLASH:
      setColor(LightColor::RED, true);
      break;
      case gazebo_traffic_light::GazeboTrafficLight_YELLOW:
      setColor(LightColor::YELLOW, false);
      break;
      case gazebo_traffic_light::GazeboTrafficLight_YELLOW_FLASH:
      setColor(LightColor::YELLOW, true);
      break;
      case gazebo_traffic_light::GazeboTrafficLight_GREEN:
      setColor(LightColor::GREEN, false);
      break;
      case gazebo_traffic_light::GazeboTrafficLight_GREEN_FLASH:
      setColor(LightColor::GREEN, true);
      break;
    }

    double time_step = (info.simTime - last_update_time_).Double();
    last_update_time_ = info.simTime;

    sequence_timestamp_ += time_step;
    if (sequence_timestamp_ >= cycle_time_) {
      sequence_timestamp_ -= cycle_time_;
    }
  }

  void GazeboTrafficLight::advanceSequence() {
    double time_thres = cycle_time_;
    for (auto it = light_sequence_.rbegin(); it != light_sequence_.rend(); ++it) {
      time_thres -= it->duration;
      if (sequence_timestamp_ >= time_thres) {
        setColor(it->color, it->flashing);
        break;
      }
    }
  }

  void GazeboTrafficLight::setColor(LightColor color, bool flashing) {
    if (flashing && ( (int)(1.25 * sequence_timestamp_) % 2) ) {
      red_switch_joint_->SetPosition(0, LIGHT_OFF_POS_);
      yellow_switch_joint_->SetPosition(0, LIGHT_OFF_POS_);
      green_switch_joint_->SetPosition(0, LIGHT_OFF_POS_);
      return;
    }
    
    switch (color) {
      case LightColor::RED:
      red_switch_joint_->SetPosition(0, LIGHT_ON_POS_);
      yellow_switch_joint_->SetPosition(0, LIGHT_OFF_POS_);
      green_switch_joint_->SetPosition(0, LIGHT_OFF_POS_);
      break;
      case LightColor::YELLOW:
      red_switch_joint_->SetPosition(0, LIGHT_OFF_POS_);
      yellow_switch_joint_->SetPosition(0, LIGHT_ON_POS_);
      green_switch_joint_->SetPosition(0, LIGHT_OFF_POS_);
      break;
      case LightColor::GREEN:
      red_switch_joint_->SetPosition(0, LIGHT_OFF_POS_);
      yellow_switch_joint_->SetPosition(0, LIGHT_OFF_POS_);
      green_switch_joint_->SetPosition(0, LIGHT_ON_POS_);
      break;
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
