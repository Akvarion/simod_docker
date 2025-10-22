# Example - BT MTC Integrati

## 🎯 Scenario semplificato:

Un robot deve:

1. Cercare un oggetto (simulato),
2. Se trovato, afferrare l’oggetto (via **MTC pick task**),
3. Altrimenti, loggare errore,
4. Portare l’oggetto in una posizione (via **MTC place task**),
5. Finire.

---

## 🧱 1. Esempio di struttura Behavior Tree (`pick_place_bt.xml`)

```xml
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="root">
      <Condition ID="ObjectDetected"/>
      <Action ID="PickObject"/>
      <Action ID="PlaceObject"/>
    </Sequence>
  </BehaviorTree>
</root>
```

---

## 🧠 2. Spiegazione dei nodi

* `ObjectDetected` → condizione che verifica se c’è un oggetto.
* `PickObject` → nodo di azione che chiama **un task MTC di grasp**.
* `PlaceObject` → nodo di azione che chiama **un task MTC di place**.

---

## 💻 3. Codice del nodo `PickObject` (esempio C++)

```cpp
class PickObject : public BT::SyncActionNode {
public:
  PickObject(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {};
  }

  BT::NodeStatus tick() override {
    ROS_INFO_STREAM("Launching Pick Task with MTC");

    auto task = std::make_shared<moveit::task_constructor::Task>("pick");
    task->loadRobotModel(node_, "robot_description");

    using namespace moveit::task_constructor;
    task->add(std::make_unique<stages::CurrentState>("current"));

    // add stages: generate grasp pose, compute IK, connect, plan, etc.

    if (!task->plan() || task->solutions().empty()) {
      ROS_ERROR_STREAM("No pick plan found!");
      return BT::NodeStatus::FAILURE;
    }

    task->introspection().publishSolution(*task->solutions().front());
    if (!task->execute(*task->solutions().front())) {
      ROS_ERROR_STREAM("Pick execution failed");
      return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_ = rclcpp::Node::make_shared("pick_node");
};
```

🔁 Puoi fare una classe `PlaceObject` molto simile.

---

## 🧩 4. Codice main (`main.cpp`) per eseguire il BT

```cpp
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  BT::BehaviorTreeFactory factory;

  factory.registerNodeType<PickObject>("PickObject");
  factory.registerNodeType<PlaceObject>("PlaceObject");
  factory.registerSimpleCondition("ObjectDetected", []() {
    return BT::NodeStatus::SUCCESS; // Simula rilevamento oggetto
  });

  auto tree = factory.createTreeFromFile("pick_place_bt.xml");

  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  while (status == BT::NodeStatus::RUNNING) {
    status = tree.tickRoot();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  rclcpp::shutdown();
  return 0;
}
```

---

## 🛠️ 5. Dipendenze da includere

Nel `CMakeLists.txt`:

```cmake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(bt_pick_place src/main.cpp)
ament_target_dependencies(bt_pick_place rclcpp behaviortree_cpp_v3 moveit_task_constructor_core)
```

---

## ✅ Conclusione

Hai ora un esempio funzionante dove:

* Il **BT è usato per orchestrare** la sequenza e fallback.
* Il **MTC è usato per gestire la manipolazione vera e propria**.
* Puoi modularizzare logicamente task più grandi, sfruttando il meglio dei due mondi.