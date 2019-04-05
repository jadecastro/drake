#include "drake/automotive/driving_command_demux.h"

#include "drake/common/autodiff.h"
#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"

namespace drake {
namespace automotive {

template <typename T>
DrivingCommandDemux<T>::DrivingCommandDemux(int num_output_ports)
    : systems::LeafSystem<T>(
          systems::SystemTypeTag<automotive::DrivingCommandDemux>{}) {
  this->DeclareInputPort(
      systems::kVectorValued,
      DrivingCommandIndices::kNumCoordinates * num_output_ports);
  for (int i = 0; i < num_output_ports; ++i) {
    auto calc_method = [this, i](const systems::Context<T>& context,
                                 systems::BasicVector<T>* vector) {
      this->CopyToOutput(context, systems::OutputPortIndex(i), vector);
    };
    this->DeclareVectorOutputPort(DrivingCommand<T>(), calc_method);
  }
}

template <typename T>
template <typename U>
DrivingCommandDemux<T>::DrivingCommandDemux(const DrivingCommandDemux<U>& other)
    : DrivingCommandDemux<T>(other.num_output_ports()) {}

template <typename T>
void DrivingCommandDemux<T>::CopyToOutput(const systems::Context<T>& context,
                                          systems::OutputPortIndex port_index,
                                          systems::BasicVector<T>* output) const {
  auto driving_command_output = dynamic_cast<DrivingCommand<T>*>(output);
  DRAKE_ASSERT(driving_command_output != nullptr);
  auto input_vector = systems::System<T>::EvalEigenVectorInput(context, 0);
  driving_command_output->set_steering_angle(input_vector(
      port_index * DrivingCommandIndices::kNumCoordinates +
      DrivingCommandIndices::kSteeringAngle));
  driving_command_output->set_acceleration(input_vector(
      port_index * DrivingCommandIndices::kNumCoordinates +
      DrivingCommandIndices::kAcceleration));
}

}  // namespace automotive
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::automotive::DrivingCommandDemux)
