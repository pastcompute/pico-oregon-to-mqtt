#ifndef DECODER_DECODE_OOK_
#define DECODER_DECODE_OOK_

#include <pico/stdlib.h>

class DecodeOOK {
public:
    enum { MAX_MESSAGE_BYTES = 48 };
protected:
    uint8_t total_bits, bits, flip, state, pos, data[MAX_MESSAGE_BYTES];

    bool overflow;

    virtual char decode (uint16_t width) =0;

public:

    enum { UNKNOWN, T0, T1, T2, T3, OK, DONE };

    DecodeOOK () { resetDecoder(); }

    bool hasOverflow() const { return overflow; }

    void clearOverflow() {
        overflow = false;
    }

    uint8_t getBits() const {
        return total_bits;
    }

    bool nextPulse (uint16_t width) {
        if (state != DONE)

            switch (decode(width)) {
                case (char)-1: resetDecoder(); break;
                case 1:  done(); break;
            }
        return isDone();
    }

    bool isDone () const { return state == DONE; }

    const uint8_t* getData (uint8_t& count) const {
        count = pos;
        return data;
    }

    void resetDecoder () {
        total_bits = bits = pos = flip = 0;
        overflow = false;
        state = UNKNOWN;
    }

    // add one bit to the packet data buffer

    virtual void gotBit (char value) {
        total_bits++;
        uint8_t *ptr = data + pos;
        *ptr = (*ptr >> 1) | (value << 7);

        if (++bits >= 8) {
            bits = 0;
            if (++pos >= sizeof data) {
                resetDecoder();
                overflow = true;
                return;
            }
        }
        state = OK;
    }

    // store a bit using Manchester encoding
    void manchester (char value) {
        flip ^= value; // manchester code, long pulse flips the bit
        gotBit(flip);
    }

    // move bits to the front so that all the bits are aligned to the end
    void alignTail (uint8_t max =0) {
        // align bits
        if (bits != 0) {
            data[pos] >>= 8 - bits;
            for (uint8_t i = 0; i < pos; ++i)
                data[i] = (data[i] >> bits) | (data[i+1] << (8 - bits));
            bits = 0;
        }
        // optionally shift uint8_ts down if there are too many of 'em
        if (max > 0 && pos > max) {
            uint8_t n = pos - max;
            pos = max;
            for (uint8_t i = 0; i < pos; ++i)
                data[i] = data[i+n];
        }
    }

    void reverseBits () {
        for (uint8_t i = 0; i < pos; ++i) {
            uint8_t b = data[i];
            for (uint8_t j = 0; j < 8; ++j) {
                data[i] = (data[i] << 1) | (b & 1);
                b >>= 1;
            }
        }
    }

    void reverseNibbles () {
        for (uint8_t i = 0; i < pos; ++i)
            data[i] = (data[i] << 4) | (data[i] >> 4);
    }

    void done () {
        while (bits)
            gotBit(0); // padding
        state = DONE;
    }
};

#endif
