#include "encoder_feedback.h"

EncoderFeedback::EncoderFeedback() : available_(false) {}

bool EncoderFeedback::begin() {
  // TODO(encoder-selection): implement the purchased encoder here. Before
  // setting available_ true, verify its voltage level, pull-up requirement,
  // signal topology, edge handling, counts/revolution, gearbox ratio and sign.
  // Do not configure the reserved D2/D3 pins based on an assumed interface.
  available_ = false;
  return available_;
}

WheelEncoderSample EncoderFeedback::sample(uint32_t now_us) {
  (void)now_us;
  const WheelEncoderSample unavailable = {0L, 0L, 0L, 0L, false};
  return unavailable;
}

bool EncoderFeedback::available() const {
  return available_;
}
