#ifndef DUMMY_PLUGIN_HPP
#define DUMMY_PLUGIN_HPP

#include <gz/sim/System.hh>

namespace gz {
namespace sim {

/// Dummy plugin for Gazebo Harmonic
class DummyPlugin : public System, public ISystemConfigure {
public:
  /// Configure the plugin
  void Configure(const Entity &entity, 
                 const std::shared_ptr<const sdf::Element> &_sdf, 
                 EntityComponentManager &ecm, 
                 EventManager &eventMgr) override;
};

}  // namespace sim
}  // namespace gz

#endif  // DUMMY_PLUGIN_HPP

