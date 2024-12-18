
// ============================================================================
// si5351.cpp :: A modified version of uSDX si5351 library
// acknowledgement to PE1NNZ <pe1nnz@amsat.org>
// ============================================================================

#include <inttypes.h>
#include "i2c0.h"
#include "si5351.h"

extern I2C0 i2c0;

// frequency calculations
inline void FAST SI5351::freq_calc_fast(int16_t df) {
  #define _MSC  0x10000
  uint32_t msb128 = _msb128 + ((int64_t)(_div * (int32_t)df) * _MSC * 128) / fxtal;
  uint16_t msp1 = _msa128min512 + msb128 / _MSC;
  uint16_t msp2 = msb128;
  // pll_regs[3:0] do not change
  pll_regs[4] = BB0(msp1);
  pll_regs[5] = ((_MSC&0xF0000)>>12);
  pll_regs[6] = BB1(msp2);
  pll_regs[7] = BB0(msp2);
}

// send register with 3 args
void SI5351::SendRegister(uint8_t reg, uint8_t* data, uint8_t n) {
  i2c0.write(SI5351_ADDR, reg, data, n);
}

// send register with 2 args
void SI5351::SendRegister(uint8_t reg, uint8_t data) {
  i2c0.write(SI5351_ADDR, reg, data);
}

// read register
uint8_t SI5351::ReadRegister(uint8_t reg) {
  uint8_t reg_val = i2c0.read(SI5351_ADDR, reg);
  return reg_val;
}

void SI5351::ms(int8_t n, uint32_t div_nom, uint32_t div_denom, uint8_t pll = PLLA, uint8_t _int = 0, uint16_t phase = 0, uint8_t rdiv = 0) {
  uint16_t msa; uint32_t msb, msc, msp1, msp2, msp3;
  // integer part
  msa = div_nom / div_denom;
  // MS divider of 4 and integer mode must be used
  if (msa == 4) _int = 1;
  // fractional part
  msb = _int ? 0 : (((uint64_t)(div_nom % div_denom)*_MSC) / div_denom);
  msc = _int ? 1 : _MSC;
  msp1 = 128*msa + 128*msb/msc - 512;
  msp2 = 128*msb - 128*msb/msc * msc;
  msp3 = msc;
  uint8_t ms_regs[8] = {
    BB1(msp3),
    BB0(msp3),
    BB2(msp1) | (rdiv<<4) | ((msa == 4)*0x0C),
    BB1(msp1),
    BB0(msp1),
    BB2(((msp3 & 0x0F0000)<<4) | msp2),
    BB1(msp2),
    BB0(msp2)
  };
  // Write to MSx
  SendRegister(n*8+42, ms_regs, 8);
  if (n < 0) {
    // MSNx PLLn: 0x40=FBA_INT; 0x80=CLKn_PDN
    SendRegister(n+16+8, 0x80|(0x40*_int));
  } else {
    // make sure to configure MS in fractional-mode
    SendRegister(n+16, ((pll)*0x20)|0x0C|3|(0x40*_int));
    SendRegister(n+165, (!_int) * phase * msa / 90);
  }
}

// make sure to configure MS in fractional-mode, perform reset afterwards
void SI5351::phase(int8_t n, uint32_t div_nom, uint32_t div_denom, uint16_t phase) {
  SendRegister(n+165, phase * (div_nom / div_denom) / 90);
}

// 0x20 reset PLLA; 0x80 reset PLLB
void SI5351::reset() {
  SendRegister(177, 0xA0);
}

// output-enable mask: CLK2=4; CLK1=2; CLK0=1
void SI5351::oe(uint8_t mask) {
  SendRegister(3, ~mask);
}

// Set a CLK0,1,2 to fout Hz with phase i, q (on PLLA)
void SI5351::freq(int32_t fout, uint16_t i, uint16_t q) {
  uint8_t rdiv = 0;
  // for higher freqs, use 3rd harmonic
  if (fout > 300000000) { i/=3; q/=3; fout/=3; }
  // divide by 128 for fout 4..500kHz
  if (fout < 500000) { rdiv = 7; fout *= 128; }
  // integer part  .. maybe 44?
  uint16_t d;
  if (fout < 30000000) d = (16 * fxtal) / fout;
  else d = (32 * fxtal) / fout;
  // for f=140..300MHz; AN619; 4.1.3, this implies integer mode
  if (fout < 3500000) d = (7 * fxtal) / fout;
  if (fout > 140000000) d = 4;
  // even numbers preferred for divider (AN619 p.4 and p.6)
  if (d % 2) d++;
  // Test if multiplier remains same for freq deviation +/- 5kHz
  // if not use different divider to make same
  if ( (d * (fout - 5000) / fxtal) != (d * (fout + 5000) / fxtal) ) d += 2;
  // Variable PLLA VCO frequency at integer multiple
  // of fout at around 27MHz*16 = 432MHz
  // spectral purity considerations
  // groups.io/g/QRPLabs/message/42662
  uint32_t fvcoa = d * fout;
  // PLLA in fractional mode
  ms(MSNA, fvcoa, fxtal);
  // Multisynth stage with integer divider but in frac
  // mode due to phase setting
  ms(MS0,  fvcoa, fout, PLLA, 0, i, rdiv);
  ms(MS1,  fvcoa, fout, PLLA, 0, q, rdiv);
  ms(MS2,  fvcoa, fout, PLLA, 0, 0, rdiv);
  if (iqmsa != (((int8_t)i-(int8_t)q)*((int16_t)(fvcoa/fout))/90)) {
    iqmsa = ((int8_t)i-(int8_t)q)*((int16_t)(fvcoa/fout))/90;
    reset();
  }
  // output enable CLK0, CLK1
  oe(0b00000011);
  _fout = fout;
  _div = d;
  _msa128min512 = fvcoa / fxtal * 128 - 512;
  _msb128=((uint64_t)(fvcoa % fxtal)*_MSC*128) / fxtal;
}

// Set a CLK2 to fout Hz (on PLLB)
void SI5351::freqb(uint32_t fout) {
  uint16_t d = (16 * fxtal) / fout;
  // even numbers preferred for divider (AN619 p.4 and p.6)
  if (d % 2) d++;
  // Variable PLLA VCO frequency at integer multiple
  // of fout at around 27MHz*16 = 432MHz
  uint32_t fvcoa = d * fout;
  ms(MSNB, fvcoa, fxtal);
  ms(MS2,  fvcoa, fout, PLLB, 0, 0, 0);
}

void SI5351::stop() {
  SendRegister(3,  0b11111111);
  SendRegister(24, 0b00000000);
  SendRegister(25, 0b00000000);
  for (uint8_t i=16; i!=24; i++) SendRegister(i, 0b10000000);
  SendRegister(187, 0);
  SendRegister(149, 0);
  SendRegister(183, 0b11010010);
}

