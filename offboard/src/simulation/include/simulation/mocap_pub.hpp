#ifndef GZ_SIM_SYSTEMS_MOCAPPUB_HH_
#define GZ_SIM_SYSTEMS_MOCAPPUB_HH_

#include <memory>
#include <gz/sim/System.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  // Forward declaration
  class MocapPubPrivate;

  /// \brief MocapPub which can be attached to a model to publish the 
  /// the ground truth pose of a model
  ///
  /// A new Gazebo Transport topic is created to send target joint torques.
  /// The default topic name is
  /// "/world/cyberzoo/model/<model_name>/vehicle_visual_odometry".
  ///
  /// This topic accepts gz::msgs::Pose values representing the current model
  /// pose.
  ///
  /// ## System Parameters
  ///
  /// `<topic>` If you wish to publish on a non-default topic you may specify it
  /// here, otherwise the controller defaults to listening on
  /// "/world/cyberzoo/model/<model_name>/vehicle_visual_odometry".
  ///
  class MocapPub
      : public System,
        public ISystemConfigure,
        public ISystemPostUpdate
  {
    /// \brief Constructor
    public: MocapPub();

    /// \brief Destructor
    public: ~MocapPub() override = default;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PostUpdate(
                const gz::sim::UpdateInfo &_info,
                const gz::sim::EntityComponentManager &_ecm) override;
            

    /// \brief Private data pointer
    private: std::unique_ptr<MocapPubPrivate> dataPtr;
  };
  }
}
}
}

#endif