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

class MidiPort
{
public:
	enum Channel : uint8_t
	{
		Channel1, Channel2, Channel3, Channel4, Channel5, Channel6, Channel7, Channel8,
		Channel9, Channel10, Channel11, Channel12, Channel13, Channel14, Channel15, Channel16,
		ChannelDefault
	};

	enum Note : uint8_t
	{
		C0, Cs0, D0, Ds0, E0, F0, Fs0, G0, Gs0, A0, As0, H0,
	};

	enum Control : uint8_t
	{
		BankSelect,
		ModulationWheel,
		BreathContoller,
		Undefined,
		FootController,
		PortamentoTime,
		DataEntryMSB,
		MainVolume,
		Balance,
		Pan
	};
public:
	explicit MidiPort(Channel channel = Channel10)
		: m_initialized(false)
		, m_defaultChannel(channel)
	{
	}

	// should be called in initialize section
	bool Initialize(uint8_t txPin)
	{
		if (txPin >= 8 && txPin <= 13)
		{
			m_serial.Initialize(txPin);
			m_initialized = true;
		}
		return m_initialized;
	}

	// input velocity range 0 .. 127
	void NoteOn(Note note, uint8_t velocity, Channel channel = ChannelDefault)
	{
		if (!m_initialized)
			return;
		if (channel == ChannelDefault)
			channel = m_defaultChannel;
		if (velocity > 127)
			velocity = 127;
		m_serial.write(0x90 + channel);
		m_serial.write(note);
		m_serial.write(velocity);
	}

	void NoteOff(Note note, Channel channel = ChannelDefault)
	{
		if (!m_initialized)
			return;
		if (channel == ChannelDefault)
			channel = m_defaultChannel;
		m_serial.write(0x80 + channel);
		m_serial.write(note);
		m_serial.write(0);

	}

	void AllNotesOff(Channel channel = ChannelDefault)
	{
		if (!m_initialized)
			return;
		if (channel == ChannelDefault)
			channel = m_defaultChannel;
		
		m_serial.write(0xB0 + channel);
		m_serial.write(123);
		m_serial.write(0);
	}

	// input value range 0 .. 127
	void ControlChange(Control control, uint8_t value, Channel channel = ChannelDefault)
	{
		if (!m_initialized)
			return;
		if (channel == ChannelDefault)
			channel = m_defaultChannel;

		if (value > 127)
			value = 127;

		m_serial.write(0xB0 + channel);
		m_serial.write(control);
		m_serial.write(value);
	}

	bool m_initialized;
	MidiOutSerial m_serial;
	Channel m_defaultChannel;
};

MidiPort g_midiPort;

void setup() {

	g_midiPort.Initialize(txPin);

	pinMode(button1Pin, INPUT);
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
		if (button1State == 1)
			g_midiPort.NoteOn(MidiPort::A0, 60);
		else
			g_midiPort.NoteOff(MidiPort::A0);
	}

	currentState = analogRead(analog1Pin);

	if (abs(currentState - slider1State) > 8)
	{
		slider1State = currentState;
		g_midiPort.ControlChange(MidiPort::ModulationWheel, slider1State / 8);
	}

	delay(10);
}


