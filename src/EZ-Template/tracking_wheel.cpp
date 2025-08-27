/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "EZ-Template/tracking_wheel.hpp"

#include "EZ-Template/util.hpp"

using namespace ez;

// ADI Encoder
tracking_wheel::tracking_wheel(std::vector<int> ports, double wheel_diameter, double distance_to_center, double ratio)
    : adi_encoder(abs(ports[0]), abs(ports[1]), util::reversed_active(ports[0])),
      smart_encoder(-1),
      pot_sensor(-1)  // <-- initialize pot_sensor
 {
  IS_TRACKER = DRIVE_ADI_ENCODER;

  distance_to_center_set(distance_to_center);
  wheel_diameter_set(wheel_diameter);
  ratio_set(ratio);
  ticks_per_rev_set(360.0);
}

// ADI Encoder in 3-wire expander
tracking_wheel::tracking_wheel(int smart_port, std::vector<int> ports, double wheel_diameter, double distance_to_center, double ratio)
    : adi_encoder({abs(smart_port), abs(ports[0]), abs(ports[1])}, util::reversed_active(ports[0])),
      smart_encoder(-1),
      pot_sensor(-1)  // <-- initialize pot_sensor
 {
  IS_TRACKER = DRIVE_ADI_ENCODER;

  distance_to_center_set(distance_to_center);
  wheel_diameter_set(wheel_diameter);
  ratio_set(ratio);
  ticks_per_rev_set(360.0);
}
// Potentiometer constructor
tracking_wheel::tracking_wheel(char port, double wheel_diameter, double distance_to_center, double ratio)
    : adi_encoder(-1, -1, false),
      smart_encoder(-1),
      pot_sensor(port) {  // <-- create ADIAnalogIn for potentiometer
  IS_TRACKER = DRIVE_POTENTIOMETER;

  distance_to_center_set(distance_to_center);
  wheel_diameter_set(wheel_diameter);
  ratio_set(ratio);

  // Pot has ~4095 steps over ~333 degrees
  double pot_ticks_per_rev = 4096.0 * (360.0 / 333.0); 
  ticks_per_rev_set(pot_ticks_per_rev);

  pot_zero = pot_sensor.get_value(); // store starting offset
}

void tracking_wheel::ticks_per_rev_set(double input) { ENCODER_TICKS_PER_REV = fabs(input); }
double tracking_wheel::ticks_per_rev_get() { return ENCODER_TICKS_PER_REV; }

void tracking_wheel::ratio_set(double input) { RATIO = fabs(input); }
double tracking_wheel::ratio_get() { return RATIO; }

void tracking_wheel::distance_to_center_flip_set(bool input) { IS_FLIPPED = input; }
bool tracking_wheel::distance_to_center_flip_get() { return IS_FLIPPED; }
void tracking_wheel::distance_to_center_set(double input) { DISTANCE_TO_CENTER = fabs(input); }
double tracking_wheel::distance_to_center_get() {
  int flipped = IS_FLIPPED ? -1 : 1;
  return DISTANCE_TO_CENTER * flipped;
}

void tracking_wheel::wheel_diameter_set(double input) { WHEEL_DIAMETER = fabs(input); }
double tracking_wheel::wheel_diameter_get() { return WHEEL_DIAMETER; }

double tracking_wheel::ticks_per_inch() {
  double c = WHEEL_DIAMETER * M_PI;
  WHEEL_TICK_PER_REV = ENCODER_TICKS_PER_REV * RATIO;
  return WHEEL_TICK_PER_REV / c;
}

double tracking_wheel::get_raw() {
  if (IS_TRACKER == DRIVE_ROTATION) {
    return smart_encoder.get_position();
  } else if (IS_TRACKER == DRIVE_POTENTIOMETER) {
    return pot_sensor.get_value() - pot_zero; // relative value
  }
  return adi_encoder.get_value();
}
double tracking_wheel::get() {
  double tpi = ticks_per_inch();
  double raw = get_raw();
  if (tpi != 0)
    return raw / tpi;
  return raw;
}
void tracking_wheel::reset() {
  if (IS_TRACKER == DRIVE_ADI_ENCODER) {
    adi_encoder.reset();
  } else if (IS_TRACKER == DRIVE_ROTATION) {
    smart_encoder.reset_position();
  } else if (IS_TRACKER == DRIVE_POTENTIOMETER) {
    pot_zero = pot_sensor.get_value(); // update zero
  }
}