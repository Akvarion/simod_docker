#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include "gazebo_link_gravity_toggle/srv/set_link_gravity.hpp"

using gazebo_link_gravity_toggle::srv::SetLinkGravity;

namespace gazebo
{
class ToggleLinkGravityPlugin : public WorldPlugin
{
public:
  void Load(physics::WorldPtr _world, sdf::ElementPtr) override
  {
    world_ = _world;

    // Inizializza ROS 2 se necessario
    if (!rclcpp::ok())
      rclcpp::init(0, nullptr);

    node_ = rclcpp::Node::make_shared("toggle_link_gravity");
    srv_ = node_->create_service<SetLinkGravity>(
      "/set_link_gravity",
      std::bind(&ToggleLinkGravityPlugin::onSetLinkGravity, this,
                std::placeholders::_1, std::placeholders::_2));

    // spinner dedicato
    exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    exec_->add_node(node_);

    // thread per lo spinner
    spinner_ = std::thread([this]()
    {
      rclcpp::Rate rate(200.0);
      while (rclcpp::ok())
      {
        exec_->spin_some();
        rate.sleep();
      }
    });

    gzdbg << "[ToggleLinkGravityPlugin] ready. Service: /set_link_gravity\n";
  }

  ~ToggleLinkGravityPlugin() override
  {
    if (exec_)
      exec_->cancel();
    if (spinner_.joinable())
      spinner_.join();
    if (rclcpp::ok())
      rclcpp::shutdown();
  }

private:
  void onSetLinkGravity(const std::shared_ptr<SetLinkGravity::Request> req,
                        std::shared_ptr<SetLinkGravity::Response> res)
  {
    if (!world_)
    {
      res->success = false;
      res->message = "World not available";
      return;
    }

    std::string model_name = req->model_name;
    std::string link_name  = req->link_name;
    // Permettiamo anche link in forma 'model::link'
    if (link_name.find("::") != std::string::npos)
    {
      // tenta di separare
      auto pos = link_name.find("::");
      model_name = link_name.substr(0, pos);
      link_name  = link_name.substr(pos + 2);
    }

    if (model_name.empty() || link_name.empty())
    {
      res->success = false;
      res->message = "model_name or link_name is empty";
      return;
    }

    physics::ModelPtr model = world_->ModelByName(model_name);
    if (!model)
    {
      res->success = false;
      res->message = std::string("Model not found: ") + model_name;
      return;
    }

    physics::LinkPtr link = model->GetLink(link_name);
    if (!link)
    {
      // a volte i link in Gazebo hanno nome fully scoped; tentiamo un giro sui link
      auto links = model->GetLinks();
      for (auto &L : links)
      {
        if (L->GetName() == link_name || L->GetScopedName() == req->link_name)
        {
          link = L;
          break;
        }
      }
    }

    if (!link)
    {
      res->success = false;
      res->message = std::string("Link not found: ") + link_name;
      return;
    }

    link->SetGravityMode(req->gravity);
    res->success = true;
    res->message = std::string("Gravity set to ") + (req->gravity ? "true" : "false")
                   + " on " + model_name + "::" + link->GetName();
  }

  physics::WorldPtr world_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr exec_;
  rclcpp::Service<SetLinkGravity>::SharedPtr srv_;
  std::thread spinner_;
};

GZ_REGISTER_WORLD_PLUGIN(ToggleLinkGravityPlugin)
}  // namespace gazebo
