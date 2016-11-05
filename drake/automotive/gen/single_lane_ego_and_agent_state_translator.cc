#include "drake/automotive/gen/single_lane_ego_and_agent_state_translator.h"

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

#include <stdexcept>

#include "drake/common/drake_assert.h"

namespace drake {
namespace automotive {

std::unique_ptr<systems::BasicVector<double>>
SingleLaneEgoAndAgentStateTranslator::AllocateOutputVector() const {
  return std::make_unique<SingleLaneEgoAndAgentState<double>>();
}

void SingleLaneEgoAndAgentStateTranslator::Serialize(
    double time, const systems::VectorBase<double>& vector_base,
    std::vector<uint8_t>* lcm_message_bytes) const {
  const auto* const vector =
      dynamic_cast<const SingleLaneEgoAndAgentState<double>*>(&vector_base);
  DRAKE_DEMAND(vector != nullptr);
  drake::lcmt_linear_car_state_t message;
  message.timestamp = static_cast<int64_t>(time * 1000);
  message.x_e = vector->x_e();
  message.v_e = vector->v_e();
  message.x_a = vector->x_a();
  message.v_a = vector->v_a();
  const int lcm_message_length = message.getEncodedSize();
  lcm_message_bytes->resize(lcm_message_length);
  message.encode(lcm_message_bytes->data(), 0, lcm_message_length);
}

void SingleLaneEgoAndAgentStateTranslator::Deserialize(
    const void* lcm_message_bytes, int lcm_message_length,
    systems::VectorBase<double>* vector_base) const {
  DRAKE_DEMAND(vector_base != nullptr);
  auto* const my_vector
    = dynamic_cast<SingleLaneEgoAndAgentState<double>*>(vector_base);
  DRAKE_DEMAND(my_vector != nullptr);

  drake::lcmt_linear_car_state_t message;
  int status = message.decode(lcm_message_bytes, 0, lcm_message_length);
  if (status < 0) {
    throw std::runtime_error("Failed to decode LCM message linear_car_state.");
  }
  my_vector->set_x_e(message.x_e);
  my_vector->set_v_e(message.v_e);
  my_vector->set_x_a(message.x_a);
  my_vector->set_v_a(message.v_a);}

}  // namespace automotive
}  // namespace drake
