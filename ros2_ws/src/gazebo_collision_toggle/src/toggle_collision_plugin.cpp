#include <memory>
#include <string>
#include <unordered_map>
#include <mutex>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include "gazebo_collision_toggle/srv/set_collision_enabled.hpp"

using gazebo_collision_toggle::srv::SetCollisionEnabled;

namespace gazebo
{
class ToggleCollisionPlugin : public WorldPlugin
{
public:
  void Load(physics::WorldPtr _world, sdf::ElementPtr) override
  {
    world_ = _world;

    if (!rclcpp::ok()) rclcpp::init(0, nullptr);

    node_ = rclcpp::Node::make_shared("toggle_collision");
    srv_  = node_->create_service<SetCollisionEnabled>(
      "/set_collision_enabled",
      std::bind(&ToggleCollisionPlugin::onSetCollisionEnabled, this,
                std::placeholders::_1, std::placeholders::_2));

    exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    exec_->add_node(node_);
    spinner_ = std::thread([this]{
      rclcpp::Rate r(200.0);
      while (rclcpp::ok()) { exec_->spin_some(); r.sleep(); }
    });

    gzdbg << "[ToggleCollisionPlugin] Ready. Service: /set_collision_enabled\n";
  }

  ~ToggleCollisionPlugin() override
  {
    if (exec_) exec_->cancel();
    if (spinner_.joinable()) spinner_.join();
    if (rclcpp::ok()) rclcpp::shutdown();
  }

private:
  void onSetCollisionEnabled(const std::shared_ptr<SetCollisionEnabled::Request> req,
                             std::shared_ptr<SetCollisionEnabled::Response> res)
  {
    if (!world_) { res->success = false; res->message = "World not available"; return; }
    if (req->model_name.empty() || req->link_name.empty()) {
      res->success = false; res->message = "model_name and link_name must be provided"; return;
    }

    auto model = world_->ModelByName(req->model_name);
    if (!model) { res->success = false; res->message = "Model not found: " + req->model_name; return; }

    // Try direct, else scan links by name
    physics::LinkPtr link = model->GetLink(req->link_name);
    if (!link) {
      auto links = model->GetLinks();
      for (auto &L : links) if (L->GetName()==req->link_name) { link = L; break; }
    }
    if (!link) { res->success = false; res->message = "Link not found: " + req->link_name; return; }

    // Gather target collisions
    std::vector<physics::CollisionPtr> targets;
    auto cols = link->GetCollisions();
    gzdbg << "[ToggleCollisionPlugin] Link '" << link->GetName()
          << "' has " << cols.size() << " collision(s):\n";
    for (auto &c : cols) {
      gzdbg << "  - name: " << c->GetName()
            << " | scoped: " << c->GetScopedName() << "\n";
      if (req->collision_name.empty()
          || c->GetName() == req->collision_name
          || c->GetScopedName() == req->collision_name) {
        targets.push_back(c);
      }
    }
    if (targets.empty()) {
      res->success = false;
      res->message = "No matching collision found on link (check names above)";
      return;
    }

    size_t n_ok = 0;
    for (auto &c : targets) {
      const std::string scoped = c->GetScopedName();
      std::lock_guard<std::mutex> lock(mtx_);
      if (!req->enable) {
        // disable: set collide bits to 0; remember a default restore value
        if (saved_bits_.find(scoped) == saved_bits_.end()) saved_bits_[scoped] = 0xFFFFu;
        c->SetCollideBits(0u);
        n_ok++;
      } else {
        // enable: restore saved or 0xFFFF
        uint16_t restore = 0xFFFFu;
        auto it = saved_bits_.find(scoped);
        if (it != saved_bits_.end()) { restore = it->second; saved_bits_.erase(it); }
        c->SetCollideBits(restore);
        n_ok++;
      }
    }

    const bool all_ok = (n_ok == targets.size());
    res->success = (n_ok > 0); // success if at least one was updated
    res->message = "Updated collisions: " + std::to_string(n_ok) + " / "
                 + std::to_string(targets.size())
                 + " (enable=" + std::string(req->enable ? "true" : "false") + ")"
                 + (all_ok ? "" : " (partial)");
  }

  // Members
  physics::WorldPtr world_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr exec_;
  rclcpp::Service<SetCollisionEnabled>::SharedPtr srv_;
  std::thread spinner_;
  std::unordered_map<std::string, uint16_t> saved_bits_;
  std::mutex mtx_;
};

GZ_REGISTER_WORLD_PLUGIN(ToggleCollisionPlugin)
} // namespace gazebo
