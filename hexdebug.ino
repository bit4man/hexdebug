#define dataInPin 4
#define selInPin  5
#define selOutPin 6
#define dataOutPin 7
#define clockPin 10

#define MAX_NOOP      0x00
#define MAX_DIG0      0x01
#define MAX_DIG1      0x02
#define MAX_DECODE    0x09
#define MAX_INTENSITY 0x0A
#define MAX_SCANLIMIT 0x0B
#define MAX_SHUTDOWN  0x0C
#define MAX_TEST      0x0F
#define MAX_FEATURE   0x0E

const PROGMEM uint8_t digits[] = {
    B01111110  // 0
  , B00110000  // 1
  , B01101101  // 2
  , B01111001  // 3
  , B00110011  // 4
  , B01011011  // 5
  , B01011111  // 6
  , B01110000  // 7
  , B01111111  // 8
  , B01111011  // 9
  , B01110111  // A
  , B00011111  // b
  , B01001110  // C
  , B00111101  // d
  , B01001111  // E
  , B01000111  // F
  , B00000000  // Blank
};

  
void max_send(byte opcode, byte data) {
  digitalWrite(clockPin, LOW);
  digitalWrite(selOutPin, LOW);
  shiftOut(dataOutPin, clockPin, MSBFIRST, opcode);
  shiftOut(dataOutPin, clockPin, MSBFIRST, data);
  digitalWrite(selOutPin, HIGH);
}

void max_reset() {
  digitalWrite(selOutPin, HIGH);
  max_send(MAX_NOOP, 0x00);  
  digitalWrite(selOutPin, HIGH);
}


void setup() {
  pinMode(dataInPin, INPUT_PULLUP);
  pinMode(selInPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(selOutPin, OUTPUT);
  pinMode(dataOutPin, OUTPUT);
  digitalWrite(selInPin, HIGH);   // inverse
  digitalWrite(selOutPin, HIGH);
  digitalWrite(clockPin, LOW);

//  Serial.begin(115200);

  max_reset();
  // Initialize MAX7219
  max_send(MAX_SHUTDOWN, 0x01);   // Normal Operation
  // max_send(MAX_FEATURE, 0x08);    // Enable SPI
  max_send(MAX_TEST, 0x00);       // Normal Operation - No TEST
  max_send(MAX_SCANLIMIT, 0x01);  // Digits 0 & 1 only
  max_send(MAX_DECODE, 0x00);     // No DECODE
  max_send(MAX_INTENSITY, 0x0A);     // 21/32 duty cycle bright
  max_send(MAX_DIG0, pgm_read_byte_near(digits + 0x10));  // Init to Blank
  max_send(MAX_DIG1, pgm_read_byte_near(digits + 0x10));  // Init to Blank
}

uint8_t oldData = 0xF5;

void loop() {
  // Get Serial input
  digitalWrite(clockPin, LOW);
  digitalWrite(selInPin, LOW);
  // LSB is on the line and will be cleared at the first raising clock
  // Grab the data now and just add it after reading the bits.
  byte lsb = digitalRead(dataInPin);
  byte inData = shiftIn(dataInPin, clockPin, LSBFIRST) << 1 | (lsb&0x01);
  
  digitalWrite(selInPin, HIGH);

  if (inData != oldData) {
    oldData = inData;
    uint8_t high = (inData & 0xF0) >> 4;
    uint8_t low = (inData & 0x0F);

    max_reset();
    max_send(MAX_DIG0, pgm_read_byte_near(digits + high));
    max_send(MAX_DIG1, pgm_read_byte_near(digits + low));
  }

  delay(50);
}
