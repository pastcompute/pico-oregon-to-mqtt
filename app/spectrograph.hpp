#ifndef POTM_SPECTROGRAPH_H_
#define POTM_SPECTROGRAPH_H_

#include <pico/stdlib.h>
#include <hardware/spi.h>
#include <assert.h>

class Spectrograph {
private:

  /// Sum of RSSI values in the current integration interval, updated each call to updateRssi()
  /// At an interval of 50us the worst case should be 510000
  /// Reset by integration()
  uint32_t runningSum_;

  /// Number of RSSI values in the current integration interval, incremented each call to updateRssi()
  /// At an interval of 50us this should be no more than 20000 in a second
  /// Reset by integration()
  uint runningCount_;

  /// Lowest RSSI byte seen in period (scale: 0 --> -0dBm, 255 --> -127.5dBm
  /// Reset by integration()
  uint8_t rssiPeakByte_;

  /// Long term mean of integration background energy (+1 means not yet set, as will be case on first call to integration())
  /// This is actually the sum of all values of runningSum / -2.F, we then divide it by periods_ to compute background
  /// Updated by integration()
  float longTermMean_ = +1;

  /// Number of times integration() has been called since reset
  uint periods_;

  /// Number of "dB" difference between the floor and the average energy in a given period for a detection
  float detectionthreshold_ = 2;
  /// True if there was a detection in the last call to integrate()
  bool detection_;
  /// Energy computed by the last call to integrate()
  float energy_;
  /// Background computed by the last call to integrate()
  float background_;
  /// Peak computed by last call to integrate()
  float peak_;

public:
  Spectrograph() { reset(); }

  void reset() {
    runningSum_ = 0;
    runningCount_ = 0;
    rssiPeakByte_ = 255;
    longTermMean_ = +1;
    periods_ = 0;
    detection_ = false;
    energy_ = -128.F;
    background_ = 0.F;
    peak_ = 0.F;
  }

  /// Each RSSI sample, update the running sum for the current integration interval
  /// We accept the RSSI as the byte value returned by the SX1231, so we can save dividing by -2.0 until the end
  void updateRssi(uint8_t rssiByte) {
    runningSum_ += rssiByte;
    runningCount_++;
    if (rssiByte < rssiPeakByte_) { rssiPeakByte_ = rssiByte; }
  }

  /// Call at end of integration time. Sums the energy in the interval, and sets detection flag if the average
  /// exceeds the background by some threshold.
  /// The following constraints apply:
  /// - the background is averaged since power up; perhaps a rolling background might be better
  void integrate() {
    // Here we are "integrating" the received "energy" above the floor
    // Of course RSSI is dB and relative to "something" but this is a useful proxy still
    // A period with no transmissions will have a lower value...
    // The value has units of dB still
    float energyProxy = runningSum_ / -2.F / runningCount_; // This is a negative number E -127.5..0
    if (longTermMean_ > 0) { // initial was +1.F
        longTermMean_ = energyProxy;
    } else {
        longTermMean_ = energyProxy + longTermMean_;
    }

    // TODO: use a geometric mean instead of average, or another mechanism to age out older average values
    float background = longTermMean_ / (periods_ + 1);

    // Now we are going to "detect" transmissions as a deviation where the total energy in this
    // integration interval is above the long term floor
    detection_ = periods_ > 1 && energyProxy - background > detectionthreshold_;
    energy_ = energyProxy;
    background_ = background;
    peak_ = rssiPeakByte_ / -2.0;

    runningSum_ = 0;
    runningCount_ = 0;
    rssiPeakByte_ = 255;

    periods_ ++;
  }

  float getBackground() const { return background_; }
  float getEnergy() const { return energy_; }
  float getPeak() const { return peak_; }
  bool getDetection() const { return detection_; }
};


#endif
