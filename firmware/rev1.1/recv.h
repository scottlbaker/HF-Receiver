
// ============================================================================
//
// recv.h   - SSB receiver library
//
// ============================================================================

#include <Arduino.h>
#include <inttypes.h>
#include "globals.h"

#ifndef RECV_H
#define RECV_H

class RECV {
  public:
    RECV();
    void begin();
    void end();
    int16_t hilb_i(int16_t);
    int16_t hilb_q(int16_t);
    int16_t agc_fast(int16_t);
    void dac_upsample(int16_t);
    void process(int16_t, int16_t);
    void init_adc();
    void init_dac();
    void set_dac_audio_enable(bool);
    void set_dac_sample_rate(uint32_t);
    void set_adc_sample_rate(uint16_t);
    int16_t get_adc(uint8_t);
    int16_t sample_corr(int16_t);
    void load_dac_audio();
    void sample_dsp();
    int16_t filter(int16_t);

};

#endif

