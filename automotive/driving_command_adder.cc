#include "drake/automotive/driving_command_adder.h"

#include "drake/common/default_scalars.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace automotive {

template <typename T>
DrivingCommandAdder<T>::DrivingCommandAdder(int num_inputs)
    : systems::LeafSystem<T>(
          systems::SystemTypeTag<automotive::DrivingCommandAdder>{}) {
  for (int i = 0; i < num_inputs; i++) {
    this->DeclareVectorInputPort(DrivingCommand<T>());
  }
  this->DeclareVectorOutputPort(
      DrivingCommand<T>(), &DrivingCommandAdder<T>::CalcSum);
}

template <typename T>
template <typename U>
DrivingCommandAdder<T>::DrivingCommandAdder(const DrivingCommandAdder<U>& other)
    : DrivingCommandAdder<T>(other.get_num_input_ports()) {}

template <typename T>
void DrivingCommandAdder<T>::CalcSum(const systems::Context<T>& context,
                                     DrivingCommand<T>* sum) const {
  sum->set_steering_angle(0.);
  sum->set_acceleration(0.);
  Eigen::VectorBlock<VectorX<T>> sum_vector = sum->get_mutable_value();
  // Sum each input port into the output.
  for (int i = 0; i < context.get_num_input_ports(); i++) {
    const DrivingCommand<T>* input_vector =
        this->template EvalVectorInput<DrivingCommand>(context, i);
    sum_vector += input_vector->get_value();
  }
}

}  // namespace automotive
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::automotive::DrivingCommandAdder)
