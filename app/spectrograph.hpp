#ifndef POTM_SPECTROGRAPH_H_
#define POTM_SPECTROGRAPH_H_

#include <pico/stdlib.h>
#include <hardware/spi.h>
#include <assert.h>

class Spectrograph {
private:

  /// Sum of RSSI values in the current integration interval
  /// At an interval of 50us the worst case should be 510000
  uint32_t runningSum_ = 0;

  /// Number of RSSI values in the current integration interval
  /// At an interval of 50us this should be no more than 20000 in a second
  uint runningCount_ = 0;

  /// Highest RSSI byte seen in period (aka lowest -dB value x 2)
  uint8_t floorByte_ = 0;

  float longTermMean_ = +1;
  uint periods_ = 0;

  bool detection_;
  float energy_;
  float background_;

public:
  Spectrograph() { }

  void reset() {
    runningSum_ = 0;
    runningCount_ = 0;
    floorByte_ = 0;
  }

  /// Each RSSI sample, update the running sum for the current integration interval
  /// We accept the RSSI as the byte value returned by the SX1231, so we can save dividing by -2.0 until the end
  void updateRssi(uint8_t rssiByte) {
    runningSum_ += rssiByte;
    runningCount_++;
    if (rssiByte > floorByte_) { floorByte_ = rssiByte; }
  }

  /// Call at end of integration time
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
    detection_ = periods_ > 1 && energyProxy - background > 2;
    energy_ = energyProxy;
    background_ = background;

    // if (periods_ > 1 && detection) {
    //     printf("%8.2f %6.1f %6.1f    ", (t1 - t0) / 1000.F, background, energyProxy);
    //     // Bin this into 3dB slots from -127
    //     int nx = (energyProxy + 127.5F) / 3.F;
    //     for (int i=0; i < nx; i++) { printf("*"); } printf("\n");
    // } else if (spectrographData.periods < 1) { 
    //     printf("%8.2f %6.1f (initial integration)\n", (t1 - t0) / 1000.F, background);
    // }
    runningSum_ = 0;
    runningCount_ = 0;
    periods_ ++;
  }

  float getBackground() const { return background_; }
  float getEnergy() const { return energy_; }
  bool getDetection() const { return detection_; }
};


#endif
