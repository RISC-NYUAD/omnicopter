#include <gz/sim/Model.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>
#include <iostream>

namespace gz {
namespace sim {

class DummyPlugin : public System, public ISystemConfigure {
public:
  void Configure(const Entity &entity, const std::shared_ptr<const sdf::Element> &/*_sdf*/,
                 EntityComponentManager &/*_ecm*/, EventManager &/*_eventMgr*/) override {
    std::cout << "Dummy Plugin Loaded for Entity: " << entity << "!\n";
  }
};

}  // namespace sim
}  // namespace gz

// Register the plugin
GZ_ADD_PLUGIN(gz::sim::DummyPlugin,
              gz::sim::System,
              gz::sim::ISystemConfigure)

