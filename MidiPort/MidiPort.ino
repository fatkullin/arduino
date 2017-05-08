const byte txPin = 10;

const byte button1Pin = 2;

const byte analog1Pin = 0;


// this class works only with PORTB output (pins 8-12)
// and only atmega 16MHz controllers
class MidiOutSerial
{
  uint8_t m_transmitBitMask;
  volatile uint8_t *m_transmitPortRegister;
public:
  void Initialize(byte txPin)
  {
    digitalWrite(txPin, HIGH);

    pinMode(txPin, OUTPUT);
    m_transmitBitMask = digitalPinToBitMask(txPin);
    uint8_t port = digitalPinToPort(txPin);
    m_transmitPortRegister = portOutputRegister(port);

  }

  void write(uint8_t value)
  {
    uint8_t one = m_transmitBitMask;
    uint8_t zero = ~m_transmitBitMask;
    uint8_t oldSREG = SREG;
    
    cli();

    //use asm to precisely calculate the moment (in cycles) 
    //when we should set PORTB output value using MIDI output speed (31250);
    //there are exactly 12 cycles + "_loop:"  between PORTB updating,
    //so "_loop:" sleeping for (16000000/31250 - 12) = 125 * 4
    __asm__ volatile (
      "in r16, %0 \n"         // r16 = PORTB
      "and r16, %[ZERO] \n"   // r16 &= zero
      "out %0, r16 \n"        // update PORTB                Cycles:
      "nop \n"                //                            [1]
      "ldi r24, 9 \n"         // r24 = 9                    [1]
      "_loop: \n"             // while (true)
      "ldi r25, 125 \n"       //   sleep 125 * 4 cycles     [1]
      "_w0: nop \n"           //        .                   [1]
      "dec r25 \n"            //        .                   [1]
      "brne _w0 \n"           //        .                   [2/1]
      "dec r24 \n"            //   r24 -= 1                 [1]
      "breq _exit \n"         //     if (r24 == 0) break;   [2/1]
      "or r16, %[ONE] \n"     //   r16 |= one               [1]
      "ldi r22, 1 \n"         //                            [1]
      "and r22, %[VALUE] \n"  //   if (value & 1            [1]
      "cpi r22, 0 \n"         //                 == 0)      [1]
      "brne _end \n"          //                            [2/1]
      "and r16, %[ZERO] \n"   //       r16 &= zero          [1]
      "_end:\n"               //                            
      "lsr %[VALUE] \n"       //   value >>= 1              [1]
      "out %0, r16 \n"        //   update PORTB             [1]
      "rjmp _loop \n"         // continue;                  [2]
      "_exit: nop\n nop\n"    //
      "nop\n nop\n nop\n"     //                            [5]
      "or r16, %[ONE] \n"     // r16 |= one                 [1]
      "out %0, r16 \n"        // update PORTB               [1]
      :
      : "I" (_SFR_IO_ADDR(PORTB)), [ZERO]"a"(zero), [ONE]"a"(one), [VALUE]"a"(value)
      : "r22", "r24", "r25", "r16"
      );


    SREG = oldSREG; // turn interrupts back on
    _delay_loop_2(128);
  }
};


MidiOutSerial mySerial;

void setup() {
  mySerial.Initialize(txPin);

  pinMode(button1Pin, INPUT);
}

//  plays a MIDI note.  Doesn't check to see that
//  cmd is greater than 127, or that data values are  less than 127:
void noteOn(int pitch, int velocity) {
  mySerial.write(0x90);
  mySerial.write(pitch);
  mySerial.write(velocity);
}

void controlChange(int value)
{
  mySerial.write(0xB0); // MIDI control change; 
  mySerial.write(1); // MIDI controller #1  (modulation wheel)
  mySerial.write(value); // MIDI controller 
}

volatile int button1State = 0;
volatile int slider1State = 0;
volatile unsigned long button1DebounceTime = 0;

const unsigned long DebounceTime = 100; //ms

void loop() {
  unsigned long currentTime = millis();

  int currentState = digitalRead(button1Pin);
  if (currentState != button1State)  //&& currentTime > button1DebounceTime
  {
    button1DebounceTime = currentTime + DebounceTime;
    button1State = currentState;
    if (button1State == 0)
      noteOn(0x1E, 0x00);
    else
      noteOn(0x1E, 0x45);
  }

  currentState = analogRead(analog1Pin);

  if (abs(currentState - slider1State) > 8)
  {
    slider1State = currentState;
    controlChange(slider1State / 8);
  }

  delay(10);
}


