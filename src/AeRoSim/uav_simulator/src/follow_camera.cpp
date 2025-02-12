#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>  // Include ROS headers
double i = 0;
namespace gazebo
{
  class FollowCamera : public ModelPlugin
  {
    private:
      physics::ModelPtr model;  // Drone model
      physics::LinkPtr camera;  // Camera link
      event::ConnectionPtr updateConnection;

    public:
      void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
      {
        // Store the model pointer
        this->model = _parent;
        ROS_ERROR("%s", this->model->GetName().c_str());        // Get the camera link inside the model
        this->camera = this->model->GetLink("camera_link");

        // Check if the camera link was found
        if (!this->camera)
        {
          gzerr << "Error: camera_link not found in " << this->model->GetName() << std::endl;
          return;
        }

        // Connect the update function to Gazebo
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&FollowCamera::OnUpdate, this));
      }

      // Called at each simulation step
      void OnUpdate()
      {
        if (!this->camera || !this->model) return;

        // Get the drone position
        ignition::math::Pose3d dronePose = this->model->WorldPose();

        // Set the camera position behind the drone (2 meters behind, 0.5m above)
        

        ignition::math::Pose3d cameraPose(
            dronePose.Pos().X()+ 0.0,
            dronePose.Pos().Y() - 0.5 - 0.4,  // Move 2 meters behind
            dronePose.Pos().Z() + i/100000 + 0.9,  // Keep the camera 0.5m above drone
            0, 0.3, 1.5708); // Keep the camera facing forward (no rotation)
          i++;
        // Apply the new position (but keep the rotation fixed)
        this->camera->SetWorldPose(cameraPose);
      }
  };
  // Register the plugin
  GZ_REGISTER_MODEL_PLUGIN(FollowCamera)
}
