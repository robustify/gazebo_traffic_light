#pragma once

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <dynamic_reconfigure/server.h>
#include <gazebo_traffic_light/GazeboTrafficLightConfig.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo {

  typedef enum {
    RED = 0, YELLOW, GREEN
  } LightColor;

  typedef std::array<physics::JointPtr, 3> TrafficLightJoints;

  struct LightSequenceEntry {
    LightColor color;
    double duration;
    bool flashing;
    LightSequenceEntry()
    : color(LightColor::RED), duration(1.0), flashing(false) {}
    LightSequenceEntry(LightColor init_color, double init_duration, bool init_flashing = false)
    : color(init_color), duration(init_duration), flashing(init_flashing) {}
  };

  class GazeboTrafficLight : public ModelPlugin {

    public:
      GazeboTrafficLight();
      virtual ~GazeboTrafficLight();

    protected:
      virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf);
      virtual void Reset();

    private:
      void OnUpdate(const common::UpdateInfo& info);
      void computeCycleTime();
      void setColor(const std::string& traffic_light_name, LightColor color, bool flashing);
      void advanceSequence(const std::string& traffic_light_name);
      void parseJointName(const std::string& input_str, std::string& traffic_light_name, std::string& joint_name);
      void reconfig(gazebo_traffic_light::GazeboTrafficLightConfig& config, uint32_t level);

      std::unique_ptr<ros::NodeHandle> n_;
      std::shared_ptr<dynamic_reconfigure::Server<gazebo_traffic_light::GazeboTrafficLightConfig> > srv_;
      gazebo_traffic_light::GazeboTrafficLightConfig cfg_;

      event::ConnectionPtr update_connection_;
      common::Time last_update_time_;
      std::map<std::string, TrafficLightJoints> traffic_lights_;

      std::vector<LightSequenceEntry> light_sequence_;
      double sequence_timestamp_;
      double cycle_time_;

      static constexpr double LIGHT_ON_POS_ = 0.101;
      static constexpr double LIGHT_OFF_POS_ = 0.0;
  };

  GZ_REGISTER_MODEL_PLUGIN(GazeboTrafficLight)

} // namespace gazebo
