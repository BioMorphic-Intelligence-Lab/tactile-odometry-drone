#include "mocap_pub.hpp"

#include <string>

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/pose.pb.h>
#include <gz/sim/components/Pose.hh>
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"

using namespace gz;
using namespace sim;
using namespace systems;

class gz::sim::systems::MocapPubPrivate
{
  /// \brief Gazebo communication node.
  public: transport::Node node;

  /// \brief Pose message publisher.
  public: transport::Node::Publisher pose_pub;

  /// \brief Frame ID of published force
  public: std::string frame_id;

  /// \brief Model interface
  public: Model model{kNullEntity};
};


//////////////////////////////////////////////////
MocapPub::MocapPub()
  : dataPtr(std::make_unique<MocapPubPrivate>())
{
}

//////////////////////////////////////////////////
void MocapPub::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);

  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "MocapPub plugin should be attached to a model "
           << "entity. Failed to initialize." << std::endl;
    return;
  }

  // Get params from SDF

  // Publishing Topic
  std::string pub_topic;
  if (_sdf->HasElement("pub_topic"))
  {
    pub_topic = transport::TopicUtils::AsValidTopic(
        _sdf->Get<std::string>("pub_topic"));

    if (pub_topic.empty())
    {
      gzerr << "Failed to create topic [" << _sdf->Get<std::string>("pub_topic") << "]"
            << std::endl;
      return;
    }
  }
  else // Define default topic
  {
    // TODO fix topic to autoload from world
    pub_topic = "/world/cyberzoo" + topicFromScopedName(this->dataPtr->model.Entity(), _ecm, false) + "/pose";
  }

  this->dataPtr->pose_pub = this->dataPtr->node.Advertise<msgs::Pose>(pub_topic);

  gzmsg << "MocapPub publishing Wrench messages on ["
    	<< pub_topic << "]" << std::endl;



  // Read out frame_id
  if (_sdf->HasElement("frame_id"))
  {
    this->dataPtr->frame_id = transport::TopicUtils::AsValidTopic(
        _sdf->Get<std::string>("frame_id"));

    if (this->dataPtr->frame_id.empty())
    {
      gzerr << "Failed to read frame_id [" << _sdf->Get<std::string>("frame_id") << "]"
            << std::endl;
      return;
    }
  }
  else // Define default frame_id
  {
    this->dataPtr->frame_id = "world";
  }
}

//////////////////////////////////////////////////
void MocapPub::PostUpdate(
    const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  GZ_PROFILE("MocapPub::PreUpdate");

  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly." << std::endl;
  }
  /* Get the Model Pose */
  auto pose = _ecm.Component<components::Pose>(this->dataPtr->model.Entity());
  if (!pose)
    return;

  /* Init the Pose Msg */
  auto pose_msg = gz::msgs::Pose();
  /* First fill in the header */
  auto header = pose_msg.mutable_header();
  auto stamp = header->mutable_stamp();
  stamp->set_sec((std::chrono::floor<std::chrono::seconds>(_info.simTime).count()));
  stamp->set_nsec(
    std::chrono::floor<std::chrono::nanoseconds>(_info.simTime -    
      std::chrono::floor<std::chrono::seconds>(_info.simTime)).count());
  auto frame = header->add_data();
  frame->set_key("frame_id");
  frame->add_value(this->dataPtr->frame_id);

  /* Then the Position */
  auto position = pose_msg.mutable_position();
  auto pos = pose->Data().Pos();
  position->set_x(pos.X());
  position->set_y(pos.Y());
  position->set_z(pos.Z());
  /* Then the Orientation */
  auto orientation = pose_msg.mutable_orientation();
  auto quat = pose->Data().Rot();
  orientation->set_x(quat.X());
  orientation->set_y(quat.Y());
  orientation->set_z(quat.Z());
  orientation->set_w(quat.W());
  
  this->dataPtr->pose_pub.Publish(pose_msg);
}


GZ_ADD_PLUGIN(MocapPub,
              System,
              MocapPub::ISystemConfigure,
              MocapPub::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(MocapPub,
                          "gz::sim::systems::MocapPub")