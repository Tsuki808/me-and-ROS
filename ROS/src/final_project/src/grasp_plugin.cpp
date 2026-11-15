// Copyright 2026 Jiayue Xie. Licensed under Apache-2.0.
// Constraint-assisted grasping: validate real model proximity and finger closure.
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Events.hh>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Bool.h>
#include <final_project/SetObjectGrasp.h>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <chrono>

namespace gazebo {
class DeliveryGrasp : public ModelPlugin {
 public:
  ~DeliveryGrasp() override {
    if (nh_) nh_->shutdown();
    queue_.disable();
    if (thread_.joinable()) thread_.join();
  }
  void Load(physics::ModelPtr model, sdf::ElementPtr) override {
    robot_ = model;
    if (!ros::isInitialized()) { gzerr << "gazebo_ros must initialize ROS first\n"; return; }
    nh_.reset(new ros::NodeHandle("/final_project"));
    nh_->setCallbackQueue(&queue_);
    service_ = nh_->advertiseService("set_grasp", &DeliveryGrasp::Request, this);
    object_service_ = nh_->advertiseService("set_object_grasp", &DeliveryGrasp::ObjectRequest, this);
    pub_ = nh_->advertise<std_msgs::Bool>("attached", 1, true);
    update_ = event::Events::ConnectWorldUpdateBegin(std::bind(&DeliveryGrasp::Update, this));
    thread_ = std::thread([this] { while (nh_->ok()) queue_.callAvailable(ros::WallDuration(.05)); });
  }
 private:
  bool Request(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
    return Submit("delivery_box", req.data, res);
  }
  bool ObjectRequest(final_project::SetObjectGrasp::Request& req, final_project::SetObjectGrasp::Response& res) {
    int mode = 0;
    nh_->getParam("operation_mode", mode);
    if (mode != 2 || (req.model_name != "delivery_box" && req.model_name != "delivery_box_red" &&
                     req.model_name != "delivery_box_green" && req.model_name != "delivery_box_blue")) {
      res.success = false; res.message = "Mode 2 and a configured box model are required"; return true;
    }
    return Submit(req.model_name, req.attach, res);
  }
  template <typename Response>
  bool Submit(const std::string& model, bool attach, Response& res) {
    std::unique_lock<std::mutex> lock(mutex_);
    requested_model_ = model;
    desired_ = attach;
    pending_ = true;
    completed_ = false;
    if (!ready_.wait_for(lock, std::chrono::seconds(3), [this] { return completed_; })) {
      pending_ = false;
      res.success = false;
      res.message = "Physics update timeout; unpause simulation";
      return true;
    }
    res.success = success_;
    res.message = message_;
    return true;
  }
  void Update() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (pending_) {
      success_ = true;
      if (joint_ && attached_model_ != requested_model_) {
        success_ = false; message_ = "A different payload is attached";
      } else if (desired_ && !joint_) {
        auto object = robot_->GetWorld()->ModelByName(requested_model_);
        auto tool = robot_->GetLink("end_effector_link");
        auto finger = robot_->GetJoint("gripper");
        auto other_finger = robot_->GetJoint("gripper_sub");
        if (!object || !tool || !finger || !other_finger) {
          success_ = false; message_ = "Required object, tool or finger missing";
        } else if (finger->Position(0) > .008 || other_finger->Position(0) > .008) {
          success_ = false; message_ = "Close the gripper before attaching";
        } else if ((object->WorldPose().Pos() - tool->WorldPose().Pos()).Length() > .035) {
          success_ = false; message_ = "Object is outside the 35 mm grasp envelope";
        } else {
          joint_ = robot_->GetWorld()->Physics()->CreateJoint("fixed", robot_);
          joint_->SetName("delivery_grasp_constraint");
          joint_->Load(tool, object->GetLink("box_link"), ignition::math::Pose3d::Zero);
          joint_->Init();
          attached_model_ = requested_model_;
          message_ = "Constraint attached after closure and proximity checks";
        }
      } else if (!desired_ && joint_) {
        joint_->Detach();
        joint_->Fini();
        joint_.reset();
        attached_model_.clear();
        message_ = "Constraint released";
      } else {
        message_ = "Already in requested state";
      }
      pending_ = false; completed_ = true; ready_.notify_one();
    }
    const auto now = robot_->GetWorld()->SimTime();
    if ((now - last_pub_).Double() > .2) {
      std_msgs::Bool msg; msg.data = static_cast<bool>(joint_); pub_.publish(msg); last_pub_ = now;
    }
  }
  physics::ModelPtr robot_;
  physics::JointPtr joint_;
  std::unique_ptr<ros::NodeHandle> nh_;
  ros::CallbackQueue queue_;
  ros::ServiceServer service_, object_service_;
  ros::Publisher pub_;
  event::ConnectionPtr update_;
  common::Time last_pub_;
  std::thread thread_;
  std::mutex mutex_;
  std::condition_variable ready_;
  bool desired_ = false, pending_ = false, completed_ = false, success_ = false;
  std::string message_;
  std::string requested_model_, attached_model_;
};
GZ_REGISTER_MODEL_PLUGIN(DeliveryGrasp)
}
