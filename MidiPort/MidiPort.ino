const byte txPin = 10;

const byte button1Pin = 2;
const byte button2Pin = 3;
const byte button3Pin = 4;

const byte analog1Pin = 0;
const byte analog2Pin = 1;
const byte analog3Pin = 2;

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
		C0 = 12, Cs0, D0, Ds0, E0, F0, Fs0, G0, Gs0, A0, As0, H0,
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
	pinMode(button2Pin, INPUT);
	pinMode(button3Pin, INPUT);
}

volatile int button1State = 0;
volatile int button2State = 0;
volatile int button3State = 0;

volatile unsigned long button1DebounceTime = 0;
volatile unsigned long button2DebounceTime = 0;
volatile unsigned long button3DebounceTime = 0;

volatile int slider1State = 0;
volatile int slider2State = 0;
volatile int slider3State = 0;


bool isDebounced(unsigned long currentTime, unsigned long debounceTime)
{
	const unsigned long DebounceTime = 100; //ms
	return currentTime > debounceTime + DebounceTime
		|| currentTime < debounceTime - DebounceTime;
}

const MidiPort::Note button1Note = MidiPort::A0;
const MidiPort::Note button2Note = MidiPort::C0;
const MidiPort::Note button3Note = MidiPort::G0;

const MidiPort::Control slider1Control = MidiPort::ModulationWheel;
const MidiPort::Control slider2Control = MidiPort::Balance;
const MidiPort::Control slider3Control = MidiPort::MainVolume;

void HandleButton(int currentState, unsigned long currentTime, unsigned long volatile* debounceTime, int volatile* buttonState, MidiPort::Note note)
{
	if (currentState != *buttonState && isDebounced(currentTime, *debounceTime))
	{
		*debounceTime = currentTime;
		*buttonState = currentState;
		if (*buttonState == 1)
			g_midiPort.NoteOn(note, 60);
		else
			g_midiPort.NoteOff(note);
	}
}

void HandleSlider(int currentState, int volatile* sliderState, MidiPort::Control control)
{
	if (abs(currentState - *sliderState) > 8)
	{
		*sliderState = currentState;
		g_midiPort.ControlChange(control, currentState / 8);
	}
}

void loop() {
	unsigned long currentTime = millis();
	int button1CurrentState = digitalRead(button1Pin);
	int button2CurrentState = digitalRead(button2Pin);
	int button3CurrentState = digitalRead(button3Pin);

	int slider1CurrentState = analogRead(analog1Pin);
	int slider2CurrentState = analogRead(analog2Pin);
	int slider3CurrentState = analogRead(analog3Pin);

	HandleButton(button1CurrentState, currentTime, &button1DebounceTime, &button1State, button1Note);
	HandleButton(button2CurrentState, currentTime, &button2DebounceTime, &button2State, button2Note);
	HandleButton(button3CurrentState, currentTime, &button3DebounceTime, &button3State, button3Note);

	HandleSlider(slider1CurrentState, &slider1State, slider1Control);
	HandleSlider(slider2CurrentState, &slider2State, slider2Control);
	HandleSlider(slider3CurrentState, &slider3State, slider3Control);

	delay((10));

}


