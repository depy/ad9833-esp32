#include <SPI.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "Button2.h"
#include "Rotary.h"

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

// Ad9833
#define FSYNC   5
#define CLK    18
#define DATA   23
#define CSDPOT 17

#define ROTARY1_PIN1 25
#define ROTARY1_PIN2 26
#define BUTTON1_PIN	 27

#define ROTARY2_PIN1 32
#define ROTARY2_PIN2 35
#define BUTTON2_PIN	 34

MCP41010* dp;
AD9833* ad9833;
LiquidCrystal_I2C lcd(0x3F,16,2);

Rotary r;
Rotary r2;
Button2 b;
Button2 b2;

byte sineChar[] = {B00000, B00000, B01000, B10101, B10101, B00010, B00000, B00000};
byte triangleChar[] = {B00000, B00000, B00100, B01010, B10001, B00000, B00000, B00000};
byte squareChar[] = {B00000, B00000, B11101, B10101, B10111, B00000, B00000, B00000};

int freq = 1000;
int currFreq = 1000;
int amplitude = 60;
int currAmplitude = 60;

int stepIndex = 2;
int steps[6] = {1,10,100,1000,10000,100000};

AD9833::Waveform mode = AD9833::SIN;
AD9833::Waveform currMode = AD9833::SIN;

int modeIndex = 0;
AD9833::Waveform modes[] = {
  AD9833::SIN,
  AD9833::TRI,
  AD9833::SQR,
};

bool update = false;
int ms = millis();

void setup() {
  Serial.begin(115200);

  SPI.begin(18, 19, 23, 5);
  SPI.setBitOrder(MSBFIRST);

  dp = new MCP41010(CSDPOT);
  ad9833 = new AD9833(FSYNC);

  ad9833->reset();
  delay(10);

  Wire.begin(21, 22);
  lcd.init();
  lcd.backlight();
  lcd.createChar(0, sineChar);
  lcd.createChar(1, triangleChar);
  lcd.createChar(2, squareChar);
  lcd.setCursor(0, 0);

  r.begin(ROTARY1_PIN1, ROTARY1_PIN2, 4);
  r.setLeftRotationHandler(adjustFreq);
  r.setRightRotationHandler(adjustFreq);

  b.begin(BUTTON1_PIN);
  b.setClickHandler(nextStepSize);
  //b.setLongClickHandler(...);

  r2.begin(ROTARY2_PIN1, ROTARY2_PIN2, 4);
  r2.setLeftRotationHandler(adjustAmplitude);
  r2.setRightRotationHandler(adjustAmplitude);

  b2.begin(BUTTON2_PIN);
  b2.setClickHandler(nextMode);
  //b2.setLongClickHandler(...);

  ad9833->signal(currFreq, currMode);

  update = true;
}

void loop() {
  r.loop();
  b.loop();
  r2.loop();
  b2.loop();

  if((millis() - ms ) > 20 && update == true) {
    ms = millis();
    updateLCD();

    if(freq != currFreq || mode != currMode) {
      ad9833->signal(freq, mode);
      currFreq = freq;
      currMode = mode;
    }

    if(amplitude != currAmplitude) {
      dp->setAmplitude(amplitude);
      currAmplitude = amplitude;
    }
  }
}

// On left or right rotation
void adjustFreq(Rotary& r) {
  int s = steps[stepIndex];
  if (r.directionToString(r.getDirection()) == "LEFT") {
    if(freq+s <= 1000000) {
      freq += s;
    }
  } else if (r.directionToString(r.getDirection()) == "RIGHT") {
    if(freq-s >= 1) {
      freq -= s;
    }
  }
  update = true;
}

// On left or right rotation
void adjustAmplitude(Rotary& r) {
  if (r.directionToString(r.getDirection()) == "LEFT") {
    if(amplitude + 5 <= 255) {
      amplitude += 5;
    }
  } else if (r.directionToString(r.getDirection()) == "RIGHT") {
    if(amplitude - 5 >= 5) {
      amplitude -= 5;
    }
  }
  update = true;
}

// Single click
void nextStepSize(Button2& btn) {
  if (stepIndex == 5) {
    stepIndex = 0;
  } else {
    stepIndex += 1;
  }
  update = true;
}

// Long click
void nextMode(Button2& btn) {
  if(modeIndex == 2) {
    modeIndex = 0;
  } else {
    modeIndex += 1;
  }
  mode = modes[modeIndex];
  //r.resetPosition();
  update = true;
}

void updateLCD() {
  lcd.clear();
  printMode();
  printStep();
  printFreq();
  printAmpl();
  update = false;
}

// Print signal mode
void printMode() {
  lcd.setCursor(0, 0);
  switch(modes[modeIndex]) {
    case AD9833::SIN: lcd.print("Sin "); lcd.write(0); break;
    case AD9833::TRI: lcd.write(1); break;
    case AD9833::SQR: lcd.write(2); break;
  }
}

// Print step size
void printStep() {
  lcd.setCursor(0, 1);
  lcd.print("Stp ");

  lcd.setCursor(4, 1);
  switch(steps[stepIndex]) {
    case 1: lcd.print("1"); break;
    case 10: lcd.print("10"); break;
    case 100: lcd.print("100"); break;
    case 1000: lcd.print("1k"); break;
    case 10000: lcd.print("10k"); break;
    case 100000: lcd.print("100k"); break;
  }
}

// Print frequency
void printFreq() {
  lcd.setCursor(6, 0);
  lcd.print(freq);
  lcd.setCursor(6 + numDigits(freq), 0);
  lcd.print("Hz");
}

// Linest in excel gave:
// m = 0.01421628336
// b = 0.01446312606
void printAmpl() {
  lcd.setCursor(9, 1);
  lcd.print("A ");
  double m = 0.01421628336;
  double b = 0.01446312606;
  float adjAmpl = (amplitude * m + b) * 1000;
  if(adjAmpl < 1000) {
    lcd.print(adjAmpl, 0);
    lcd.print("mV  ");
  } else {
    lcd.print(adjAmpl/1000, 2);
    lcd.print("V ");
  }
  lcd.print(amplitude);
}

int numDigits(int num) {
  int digits = 0;
  while(num) {
    num /= 10;
    digits++;
  }
  return digits;
}
