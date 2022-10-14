#ifndef POTM_LACROSSE_H_
#define POTM_LACROSSE_H_

#include "decoder/DecodeOOK.hpp"

// Attempt to decode, based on the protocol for the TX141B documented in rtl_433

class LacrosseDecoder : public DecodeOOK {
private:
  enum { DECODE_CONTINUE = 0 , DECODE_DONE = 1, DECODE_RESET = -1 };

  enum : uint16_t { T_SYNC = 833, T_SHORT = 208, T_LONG=417 };

  uint16_t syncWidthLo;
  uint16_t syncWidthHi;
  uint16_t shortWidthLo;
  uint16_t shortWidthHi;
  uint16_t longWidthLo;
  uint16_t longWidthHi;

  uint16_t prevWidth;

  int8_t t3Pending;

public:
  int8_t doneReason;

  LacrosseDecoder(uint8_t tolerancePercent = 33) {
    prevWidth = 0;
    t3Pending = -1;
    init(tolerancePercent);
  }
  inline void setToleranceRange(uint16_t t_us, uint8_t tolerancePercent, uint16_t& lo, uint16_t& hi) {
    uint16_t tolerance = (uint32_t)t_us * tolerancePercent / 100u;
    lo = t_us - tolerance;
    hi = t_us + tolerance;
  }
  void init(uint8_t tolerancePercent) {
    uint16_t tolerance;
    setToleranceRange(T_SYNC, tolerancePercent, syncWidthLo, syncWidthHi);
    syncWidthHi = T_SYNC + 70; // try and filter out oregon
    setToleranceRange(T_SHORT, tolerancePercent, shortWidthLo, shortWidthHi);
    setToleranceRange(T_LONG, tolerancePercent, longWidthLo, longWidthHi);
    longWidthLo += 50; // dont overcook the hi range
    shortWidthHi += 50; // and give short a tad more
    printf("%d %d %d %d %d %d\n", syncWidthLo, syncWidthHi, shortWidthLo, shortWidthHi, longWidthLo, longWidthHi);
  }

  // First, the preamble is 4x 833 us high, 833us low. Allow some margin (25% each side)
  virtual char decode (uint16_t width) {
    if (syncWidthLo < width && width < syncWidthHi) {
      // potential preamble. If we are unknown, start counting flips. If we have 8 in a row, this is a candidate preamble
      prevWidth = width;
      switch (state) {
        case UNKNOWN:
          flip++;
          if (flip == 7) {
            state = T0;
          }
          return DECODE_CONTINUE;
        case T0:
          flip++;
          if (flip == 8) {
            // We assume this was a 0...
            // now we should get short-long or long-short's x 40
            t3Pending = -1;
            doneReason = 0;
            return DECODE_CONTINUE;
          }
          // too many... note, dont print debug here, because it will always pickup the oregon OOK...
          return DECODE_RESET;
        case T1:
          // fall through
        case T2:
          // if T1 or T2, we ran past the end... (there can be a post-amble)
          //printf("Z %d %d %d %d %d\n", state, width, prevWidth, flip, total_bits);
          if (total_bits >= 32) {
            doneReason = 1;
            return DECODE_DONE;
          }
          return DECODE_RESET;
        default:
          panic("Invalid state (preamble)");
          break;
      }
    }
    if (state == T0) { // assume width is long or short...
      //printf("~");
    }

    // Now we have a fixed clock of 625uS, with short-long == 0 and long-short ==1
    // short 208us, long 417us
    bool shortPulse = shortWidthLo < width && width < shortWidthHi;
    bool longPulse = longWidthLo < width && width < longWidthHi;
    if (!longPulse && !shortPulse) {
      // If we have enough bits, we are done...
      //if (state != UNKNOWN) { printf("X: %d %d %d %d %d\n", width, prevWidth, state, total_bits, flip); }
      if (total_bits >= 32) {
        doneReason = 2;
        return DECODE_DONE;
      }
      return DECODE_RESET;
    }
    // We only get here now, if it is a proper short or long pulse
    // If the state is UNKNOWN, then we never got through the preamble
    if (state == UNKNOWN) {
      return DECODE_RESET;
    }
    flip++; // keep counting for our debugging
    prevWidth = 0;
    switch (state) {
      case T0:
        // This will be the initial state after Preamble, and also after each 625 uS cycle
        prevWidth = width;
        if (shortPulse) {
          // expect to be followed by a longPulse
          state = T1;
        } else if (longPulse) {
          // expect to be followed by a shortPulse
          state = T2;
        }
        return DECODE_CONTINUE;
      case T1:
        // Short, hopefully followed by a long
        if (longPulse) {
          gotBit(0);
          state = T0;
          return DECODE_CONTINUE;
        }
        break;
      case T2:
        // Long, hopefully followed by a short
        if (shortPulse) {
          gotBit(1);
          state = T0;
          return DECODE_CONTINUE;
        }
        break;
      //   // Long followed by long, _could_ be long-short then short-long where short-short is detected as long.  But we dont know the polarity...
      //   t3Pending = 0;
      //   state = T3;
      //   return DECODE_CONTINUE;
      // case T3:
      //   // long-short followed by short-long detected as long-long-long 
      //   if (t3Pending == 0 && longPulse) {
      //     state = T0;
      //     gotBit(1);
      //     gotBit(0);
      //     return DECODE_CONTINUE;
      //   }
      //   // short-long followed by short-long detected as short-long-long 
      //   if (t3Pending == 1 && longPulse) {
      //     state = T0;
      //     gotBit(1);
      //     gotBit(0);
      //     return DECODE_CONTINUE;
      //   }
      //   // long-short followed by short-long || short-short, short-short
      default:
        panic("Invalid state");
        break;
    }
    printf("F %d %d %d %d %d %d %d\n", width, prevWidth, state, total_bits, flip, longPulse, shortPulse);
    return DECODE_RESET;
  }
};

#endif
