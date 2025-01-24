#include <SPI.h>

/* ===== From MCP41010 data sheet =====
Command byte:
  [X X X C1 C0 X P1* P0]

  Command selection bits (C1, C0)
    0 0 => None: No Command will be executed.
    
    0 1 => Write data
      Write the data contained in Data Byte to the
      potentiometer(s) determined by the potentiometer selection bits.

    1 0 => Shudown
      Potentiometer(s) determined by potentiometer selection bits will enter Shutdown Mode.
      Data bits for this command are 'don’t cares'
    
    1 1 => None: No Command will be executed.

  Potentiometer selection bits (P1*, P0)
    0 0 => Dummy Code: Neither Potentiometer affected.
    0 1 => Command executed on Potentiometer 0.
    1 0 => Command executed on Potentiometer 1.
    1 1 => Command executed on both Potentiometers.
*/
class MCP41010 {
  uint8_t writeCommand = B00010001;   // Command byte
  uint8_t chipSelect;                 // Chip select pin

public:
  MCP41010(uint8_t csPin) {
    chipSelect = csPin;
    pinMode(chipSelect, OUTPUT);
  }

  void setAmplitude(uint8_t a) {
    if(a < 0) { a = 0; }
    if(a > 255) { a = 255; }

    digitalWrite(chipSelect, LOW);    // Select the MCP41010 chip
    SPI.setDataMode(0);               // Set SPI data mode to 0 (as specified in the data sheet)
    SPI.transfer(writeCommand);       // Send "write" command byte
    SPI.transfer(a);                  // Send data
    digitalWrite(chipSelect, HIGH);   // Deselect the MCP 41010 chip
  }
};

class AD9833 {

public:
  enum Waveform { SIN, TRI, SQR };

  AD9833(uint8_t fsyncPin) {
    chipSelectPin = fsyncPin;
    pinMode(chipSelectPin, OUTPUT);
  }

  void reset() {
    uint16_t value = 0 | CR_RESET;
    writeControlReg(value);
  }

  void signal(uint32_t freq, Waveform w) {
    prepare();
    setFreq(0, freq);
    setWaveForm(w);
  }

private:
  // Control register bits
  const uint16_t CR_B28 =     (1 << 13); // Allows to load whole word into a frequency register in two consecutive writes
  const uint16_t CR_FSELECT = (1 << 11); // Defines if FREQ0 register or the FREQ1 register is used in the phase accumulator
  const uint16_t CR_PSELECT = (1 << 10); // Defines if PHASE0 register or the PHASE1 register data is added to the output of the phase accumulator
  const uint16_t CR_RESET =   (1 << 8);  // Reset bit
  const uint16_t CR_OPBITEN = (1 << 5);  // OPBITEN bit
  const uint16_t CR_DIV2 =    (1 << 3);  // DIV2 bit
  const uint16_t CR_MODE =    (1 << 1);  // MODE bit

  const uint16_t SIN_WAVE = 0x2000;
  const uint16_t TRI_WAVE = 0x2002;
  const uint16_t SQR_WAVE = 0x2028;

  // Other stuff
  const float refFreq = 25000000;  // 25 Mhz reference freqency to calculate the data value
  uint8_t chipSelectPin;

  uint32_t freqData(float desiredFreq) {
    float _2pow28 = 268435456;
    return round((desiredFreq * _2pow28) / refFreq);
  }

  uint16_t phaseData(float desiredPhase) {
    if(desiredPhase > 360) { desiredPhase = 0; }
    if(desiredPhase < 0) { desiredPhase = 360; }

    return desiredPhase * (4095.0 / 360.0);
  }
  
  void writeRegister(uint16_t data) { 
    SPI.setDataMode(SPI_MODE2);
    
    digitalWrite(chipSelectPin, LOW);   // Set FSYNC low before writing to AD9833 registers
    delayMicroseconds(10);              // Give AD9833 time to get ready to receive data.
    
    SPI.transfer(highByte(data));        // Each AD9833 register is 32 bits wide and each 16
    SPI.transfer(lowByte(data));         // bits has to be transferred as 2 x 8-bit bytes.

    digitalWrite(chipSelectPin, HIGH);          //Write done. Set FSYNC high
  }

  void writeControlReg(uint16_t value) {
    uint16_t mask = 0b0011111111111111;
    writeRegister(value & mask);
  }

  void prepare() {
    uint16_t resetAndSetDoubleWrite = CR_B28 | CR_RESET;
    writeControlReg(resetAndSetDoubleWrite); 
  }

  void setPhase(uint16_t selectedReg, float phase) {
    uint16_t dataMask = 0b0000111111111111;
    uint16_t regSelect;

    if(selectedReg == 0) {
      regSelect = 0b1100000000000000;
    } else {
      regSelect = 0b1110000000000000;
    }

    uint16_t value = phaseData(phase);
    uint16_t data = regSelect | (value & dataMask);
    writeRegister(data);
  }

  void setFreq(uint8_t selectedReg, float freq) {
    uint32_t dataMask = 0b1111111111111111111111111111;
    uint16_t regSelect;

    if(selectedReg == 0) {
      regSelect = 0b0100000000000000;
    } else {
      regSelect = 0b1000000000000000;
    }
    uint32_t value = freqData(freq); 
    uint32_t data = value & dataMask;
    uint16_t LSB = regSelect | (data & 0b11111111111111); // Get lower 14 bits from data and add freqency register selection bits (15 and 14) on top
    uint16_t MSB = regSelect | (data >> 14);              // Get upper 14 bits from data and add freqency register selection bits (15 and 14) on top

    writeRegister(LSB);
    writeRegister(MSB);
  }

  void setWaveForm(Waveform w) {
    switch(w) {
      case SIN  : writeRegister(SIN_WAVE);   break;
      case TRI  : writeRegister(TRI_WAVE);   break;
      case SQR  : writeRegister(SQR_WAVE);   break;
    }
  }
};

#define FSYNC 5                       // Standard SPI pins for the AD9833 waveform generator.
#define CLK 18                         // CLK and DATA pins are shared with the TFT display.
#define DATA 23
#define CSDPOT 22

MCP41010* dp;
AD9833* ad9833;

void setup() {
  Serial.begin(115200);

  SPI.begin(18, 19, 23, 5);
  SPI.setBitOrder(MSBFIRST);

  dp = new MCP41010(CSDPOT);
  ad9833 = new AD9833(FSYNC);

  ad9833->reset();
  delay(10);

  ad9833->prepare();
  ad9833->setFreq(0, 400);
  ad9833->setWaveForm(AD9833::SIN);

  ad9833->prepare();
  ad9833->setFreq(1, 800);
  ad9833->setWaveForm(AD9833::SIN);
}

void loop() {
  delay(2000);
  ad9833->writeControlReg(0);

  delay(2000);
  ad9833->writeControlReg((1 << 11));

  // delay(1000);
  // dp->setAmplitude(255);
  // ad9833->signal(1000, 0, AD9833::SIN);

  // delay(1000);
  // ad9833->signal(1000, 90, AD9833::SIN);

  // delay(1000);
  // dp->setAmplitude(255);
  // ad9833->signal(1000000, 0, AD9833::SIN);

  // delay(1000);
  // dp->setAmplitude(255);
  // ad9833->signal(1000000, 0, AD9833::TRI);

  // delay(1000);
  // dp->setAmplitude(32);
  // ad9833->signal(1000000, 0, AD9833::SQR);
}
