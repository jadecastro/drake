#include "drake/systems/framework/context_base.h"

#include <string>
#include <typeinfo>

namespace drake {
namespace systems {

std::unique_ptr<ContextBase> ContextBase::Clone() const {
  std::unique_ptr<ContextBase> clone_ptr(CloneWithoutPointers());

  // Verify that the most-derived Context didn't forget to override
  // CloneWithoutPointers().
  const ContextBase& source = *this;  // Deref here to avoid typeid warning.
  ContextBase& clone = *clone_ptr;
  DRAKE_ASSERT(typeid(source) == typeid(clone));

  // Create a complete mapping of tracker pointers.
  DependencyTracker::PointerMap tracker_map;
  BuildTrackerPointerMap(clone, &tracker_map);

  // Then do a pointer fixup pass.
  clone.FixTrackerPointers(source, tracker_map);
  return clone_ptr;
}

ContextBase::~ContextBase() {}

void ContextBase::DisableCaching() const {
  cache_.DisableCaching();
  const int n = do_num_subcontexts();
  for (SubsystemIndex i(0); i < n; ++i)
    do_get_subcontext(i).DisableCaching();
}

void ContextBase::EnableCaching() const {
  cache_.EnableCaching();
  const int n = do_num_subcontexts();
  for (SubsystemIndex i(0); i < n; ++i)
    do_get_subcontext(i).EnableCaching();
}

void ContextBase::SetAllCacheEntriesOutOfDate() const {
  cache_.SetAllEntriesOutOfDate();
  const int n = do_num_subcontexts();
  for (SubsystemIndex i(0); i < n; ++i)
    do_get_subcontext(i).SetAllCacheEntriesOutOfDate();
}

std::string ContextBase::GetSystemPathname() const {
  std::vector<const ContextBase*> path_to_root{this};
  while (const ContextBase* parent = path_to_root.back()->get_parent_base())
    path_to_root.push_back(parent);
  std::string path;
  std::for_each(path_to_root.rbegin(), path_to_root.rend(),
                [&path](const ContextBase* node) {
                  path += "/" + node->GetSystemName();
                });
  return path;
}

FreestandingInputPortValue& ContextBase::FixInputPort(
    int index, std::unique_ptr<AbstractValue> value) {
  auto freestanding =
      std::make_unique<FreestandingInputPortValue>(std::move(value));
  FreestandingInputPortValue& freestanding_ref = *freestanding;
  SetFixedInputPortValue(InputPortIndex(index), std::move(freestanding));
  return freestanding_ref;
}

void ContextBase::AddInputPort(InputPortIndex expected_index,
                               DependencyTicket ticket) {
  DRAKE_DEMAND(expected_index.is_valid() && ticket.is_valid());
  DRAKE_DEMAND(expected_index == get_num_input_ports());
  DRAKE_DEMAND(input_port_tickets_.size() == input_port_values_.size());
  auto& ui_tracker = graph_.CreateNewDependencyTracker(
      ticket, "u_" + std::to_string(expected_index));
  input_port_values_.emplace_back(nullptr);
  input_port_tickets_.emplace_back(ticket);
  auto& u_tracker = graph_.get_mutable_tracker(
      DependencyTicket(internal::kAllInputPortsTicket));
  u_tracker.SubscribeToPrerequisite(&ui_tracker);
}

void ContextBase::SetFixedInputPortValue(
    InputPortIndex index,
    std::unique_ptr<FreestandingInputPortValue> port_value) {
  DRAKE_DEMAND(0 <= index && index < get_num_input_ports());
  DRAKE_DEMAND(port_value != nullptr);

  DependencyTracker& port_tracker =
      get_mutable_tracker(input_port_tickets()[index]);
  FreestandingInputPortValue* old_value =
      input_port_values_[index].get_mutable();

  if (old_value != nullptr) {
    // All the dependency wiring is already in place.
    port_value->set_ticket(old_value->ticket());
  } else {
    // Create a new tracker and subscribe to it.
    DependencyTracker& value_tracker = graph_.CreateNewDependencyTracker(
        "Value for fixed input port " + std::to_string(index));
    port_value->set_ticket(value_tracker.ticket());
    port_tracker.SubscribeToPrerequisite(&value_tracker);
  }

  // Fill in the FreestandingInputPortValue object and install it.
  port_value->set_input_port_index(index);
  port_value->set_owning_subcontext(this);
  input_port_values_[index] = std::move(port_value);

  // Invalidate anyone who cares about this input port.
  port_tracker.NoteValueChange(start_new_change_event());
}

// Set up trackers for independent sources: time, accuracy, state, parameters,
// and input ports.
void ContextBase::CreateBuiltInTrackers() {
  DependencyGraph& graph = graph_;
  // This is the dummy "tracker" used for constants and anything else that has
  // no dependencies on any Context source. Ignoring return value.
  graph.CreateNewDependencyTracker(
      DependencyTicket(internal::kNothingTicket), "nothing");

  // Allocate trackers for time, accuracy, q, v, z.
  auto& time_tracker = graph.CreateNewDependencyTracker(
      DependencyTicket(internal::kTimeTicket), "t");
  auto& accuracy_tracker = graph.CreateNewDependencyTracker(
      DependencyTicket(internal::kAccuracyTicket), "accuracy");
  auto& q_tracker = graph.CreateNewDependencyTracker(
      DependencyTicket(internal::kQTicket), "q");
  auto& v_tracker = graph.CreateNewDependencyTracker(
      DependencyTicket(internal::kVTicket), "v");
  auto& z_tracker = graph.CreateNewDependencyTracker(
      DependencyTicket(internal::kZTicket), "z");

  // Continuous state xc depends on q, v, and z.
  auto& xc_tracker = graph.CreateNewDependencyTracker(
      DependencyTicket(internal::kXcTicket), "xc");
  xc_tracker.SubscribeToPrerequisite(&q_tracker);
  xc_tracker.SubscribeToPrerequisite(&v_tracker);
  xc_tracker.SubscribeToPrerequisite(&z_tracker);

  // Allocate the "all discrete variables" xd tracker. The associated System is
  // responsible for allocating the individual discrete variable group xdᵢ
  // trackers and subscribing this one to each of those.
  auto& xd_tracker = graph.CreateNewDependencyTracker(
      DependencyTicket(internal::kXdTicket), "xd");

  // Allocate the "all abstract variables" xa tracker. The associated System is
  // responsible for allocating the individual abstract variable xaᵢ
  // trackers and subscribing this one to each of those.
  auto& xa_tracker = graph.CreateNewDependencyTracker(
      DependencyTicket(internal::kXaTicket), "xa");

  // The complete state x={xc,xd,xa}.
  auto& x_tracker = graph.CreateNewDependencyTracker(
      DependencyTicket(internal::kXTicket), "x");
  x_tracker.SubscribeToPrerequisite(&xc_tracker);
  x_tracker.SubscribeToPrerequisite(&xd_tracker);
  x_tracker.SubscribeToPrerequisite(&xa_tracker);

  // Allocate the "all parameters" p tracker. The associated System is
  // responsible for allocating the individual numeric parameter pnᵢ and
  // abstract paraemter paᵢ trackers and subscribing this one to each of those.
  auto& p_tracker = graph.CreateNewDependencyTracker(
      DependencyTicket(internal::kAllParametersTicket), "p");

  // Allocate the "all input ports" u tracker. The associated System is
  // responsible for allocating the individual input port uᵢ
  // trackers and subscribing this one to each of those.
  auto& u_tracker = graph.CreateNewDependencyTracker(
      DependencyTicket(internal::kAllInputPortsTicket), "u");

  // Allocate the "all sources" tracker. The complete list of known sources
  // is t,a,x,p,u. Note that cache entries are not included. Under normal
  // operation that doesn't matter because cache entries are invalidated only
  // when one of these source values changes. Any computation that has
  // declared "all sources" dependence will also have been invalidated for the
  // same reason so doesn't need to explicitly list cache entries.
  auto& all_sources_tracker = graph.CreateNewDependencyTracker(
      DependencyTicket(internal::kAllSourcesTicket), "all sources");
  all_sources_tracker.SubscribeToPrerequisite(&time_tracker);
  all_sources_tracker.SubscribeToPrerequisite(&accuracy_tracker);
  all_sources_tracker.SubscribeToPrerequisite(&x_tracker);
  all_sources_tracker.SubscribeToPrerequisite(&p_tracker);
  all_sources_tracker.SubscribeToPrerequisite(&u_tracker);

  // Allocate kinematics trackers to provide a level of abstraction from the
  // specific state variables that are use to represent configuration and
  // rate of change of configuration. For example, a kinematics cache entry
  // should depend on configuration regardless of whether we use continuous or
  // discrete variables. And it should be possible to switch between continuous
  // and discrete representations without having to change the specified
  // dependency, which remains "configuration" either way.

  // Should track changes to configuration regardless of how represented. The
  // default is that the continuous "q" variables represent the configuration.
  auto& configuration_tracker = graph.CreateNewDependencyTracker(
      DependencyTicket(internal::kConfigurationTicket), "configuration");
  // This default subscription must be changed if configuration is not
  // represented by q in this System.
  configuration_tracker.SubscribeToPrerequisite(&q_tracker);

  // Should track changes to configuration time rate of change (i.e., velocity)
  // regardless of how represented. The default is that the continuous "v"
  // variables represent the configuration rate of change.
  auto& velocity_tracker = graph.CreateNewDependencyTracker(
  DependencyTicket(internal::kVelocityTicket), "velocity");
  // This default subscription must be changed if velocity is not
  // represented by v in this System.
  velocity_tracker.SubscribeToPrerequisite(&v_tracker);

  // This tracks configuration & velocity regardless of how represented.
  auto& kinematics_tracker = graph.CreateNewDependencyTracker(
      DependencyTicket(internal::kKinematicsTicket), "kinematics");
  kinematics_tracker.SubscribeToPrerequisite(&configuration_tracker);
  kinematics_tracker.SubscribeToPrerequisite(&velocity_tracker);

  auto& xcdot_tracker = graph.CreateNewDependencyTracker(
      DependencyTicket(internal::kXcdotTicket), "xcdot");
  // TODO(sherm1) Connect to cache entry.
  unused(xcdot_tracker);

  auto& xdhat_tracker = graph.CreateNewDependencyTracker(
      DependencyTicket(internal::kXdhatTicket), "xdhat");
  // TODO(sherm1) Connect to cache entry.
  unused(xdhat_tracker);
}

void ContextBase::BuildTrackerPointerMap(
    const ContextBase& clone,
    DependencyTracker::PointerMap* tracker_map) const {
  // First map the pointers local to this context.
  graph_.AppendToTrackerPointerMap(clone.get_dependency_graph(),
                                   &(*tracker_map));
  // Then recursively ask our descendants to add their information to the map.
  for (SubsystemIndex i(0); i < num_subcontexts(); ++i)
    get_subcontext(i).BuildTrackerPointerMap(clone.get_subcontext(i),
                                             &(*tracker_map));
}

void ContextBase::FixTrackerPointers(
    const ContextBase& source,
    const DependencyTracker::PointerMap& tracker_map) {
  // First repair pointers local to this context.
  graph_.RepairTrackerPointers(source.get_dependency_graph(), tracker_map, this,
                               &cache_);
  // Cache and FixedInputs only need their back pointers set to this.
  cache_.RepairCachePointers(this);
  for (auto& fixed_input : input_port_values_) {
    if (fixed_input != nullptr) fixed_input->set_owning_subcontext(this);
  }

  // Then recursively ask our descendants to repair their pointers.
  for (SubsystemIndex i(0); i < num_subcontexts(); ++i)
    get_mutable_subcontext(i).FixTrackerPointers(source.get_subcontext(i),
                                                 tracker_map);
}

}  // namespace systems
}  // namespace drake
