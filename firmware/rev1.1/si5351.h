
// ============================================================================
// si5351.h :: A modified version of uSDX si5351 library
// acknowledgement to PE1NNZ <pe1nnz@amsat.org>
// ============================================================================

#ifndef SI5351_h
#define SI5351_h

// 0x60 for SI5351A-B-GT
// 0x60 for SI5351A-B04771-GT
// 0x62 for SI5351A-B-04486-GT
// 0x6F for SI5351A-B02075-GT

#define SI5351_ADDR   0x60

class SI5351 {
public:

  volatile int32_t  _fout;
  volatile uint8_t  _div;
  volatile uint16_t _msa128min512;
  volatile uint32_t _msb128;
  volatile uint8_t  pll_regs[8];
  volatile uint32_t fxtal;
  volatile int32_t  fxadj;

  int16_t iqmsa; // to detect a need for a PLL reset
  enum ms_t { PLLA=0, PLLB=1, MSNA=-2, MSNB=-1, MS0=0, MS1=1, MS2=2, MS3=3, MS4=4, MS5=5 };

  // Bash byte x of int32_t
  #define BB0(x) ((uint8_t)(x))
  #define BB1(x) ((uint8_t)((x)>>8))
  #define BB2(x) ((uint8_t)((x)>>16))

  #define FAST __attribute__((optimize("Ofast")))

  inline void FAST freq_calc_fast(int16_t df);
  inline void SendPLLRegister();

  void i2c_write(uint8_t, uint8_t, uint8_t);
  void bulk_write(uint8_t, uint8_t, uint8_t*, uint8_t);
  void SendRegister(uint8_t, uint8_t*, uint8_t);
  void SendRegister(uint8_t, uint8_t);
  uint8_t ReadRegister(uint8_t);
  void ms(int8_t, uint32_t, uint32_t, uint8_t, uint8_t, uint16_t, uint8_t);
  void phase(int8_t, uint32_t, uint32_t, uint16_t);
  void reset();
  void oe(uint8_t);
  void freq(int32_t, uint16_t, uint16_t);
  void freqb(uint32_t);
  void stop();

};

#endif

