#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <chrono>
#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <thread>

namespace gazebo {
class ModelPush : public ModelPlugin {
   public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
        // Store the pointer to the model
        this->model = _parent;
        frame_no = 0;

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&ModelPush::OnUpdate, this));

        ROS_WARN("Loaded ModelPush Plugin with parent...%s", this->model->GetName().c_str());
    }

    // Called by the world update start event
   public:
    void OnUpdate() {
        // Apply a small linear velocity to the model.
        frame_no += 1;
        double x_vel = 0;
        if (frame_no > 20000)  // The object will start to move after 20 seconds. 
            x_vel = -0.5;      // Velocity is of the object is set to 0.5 m/s
        this->model->SetLinearVel(ignition::math::Vector3d(x_vel, 0, 0));
    }

    // Pointer to the model
   private:
    int frame_no;
    physics::ModelPtr model;

    // Pointer to the update event connection
   private:
    event::ConnectionPtr updateConnection;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}  // namespace gazebo
