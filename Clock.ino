//**************************************************************//
//  Name    : shiftOutCode, Dual Binary Counters                 //
//  Author  : Carlyn Maw, Tom Igoe                               //
//  Date    : 25 Oct, 2006                                       //
//  Version : 1.0                                                //
//  Notes   : Code for using a 74HC595 Shift Register            //
//          : to count from 0 to 255                             //
//**************************************************************//


#include <ThreeWire.h>
#include <RtcDS1302.h>
#include <Bounce2.h>

#define DATA_PIN  2  // Pin connected to DS of 74HC595
#define LATCH_PIN 3  // Pin connected to ST_CP of 74HC595
#define CLOCK_PIN 4 // Pin connected to SH_CP of 74HC595
#define OE_PIN 5 // Pin connected to OE of 74HC595
#define PB_PIN 6

#define RTC_CLK 7
#define RTC_DAT 8
#define RTC_RST 9

// How many of the shift registers
#define NUM_SHIFT_REGS 4

ThreeWire rtcWires(RTC_DAT, RTC_CLK, RTC_RST);
RtcDS1302<ThreeWire> rtc(rtcWires);

Bounce pb(PB_PIN, 50);

const uint8_t numOfRegisterPins = NUM_SHIFT_REGS * 8;

bool registers[numOfRegisterPins];

static void clearRegisters();
static void writeRegisters();
static void setRegisterPin(int index, int value);
void shiftOut(uint8_t data);

void setup() {
	Serial.begin(115200);

	pinMode(DATA_PIN, OUTPUT);
	pinMode(CLOCK_PIN, OUTPUT);
	pinMode(LATCH_PIN, OUTPUT);

	// Pull OE to ground
	pinMode(OE_PIN, OUTPUT);
	digitalWrite(OE_PIN, LOW);

	clearRegisters();
	writeRegisters();

	rtc.Begin();

	rtc.SetIsRunning(true);
	rtc.SetIsWriteProtected(false);
	auto now = RtcDateTime(__DATE__, __TIME__);
	printDateTime(now);
	rtc.SetDateTime(RtcDateTime(__DATE__, __TIME__));
}

void printDateTime(const RtcDateTime& dt)
{
	char datestring[20];

	snprintf_P(datestring,
		20,
		PSTR("%02u/%02u/%04u %02u:%02u:%02u"),
		dt.Month(),
		dt.Day(),
		dt.Year(),
		dt.Hour(),
		dt.Minute(),
		dt.Second());
	Serial.println(datestring);
}

bool toggle = false;

int i = 0;
int pwm = 0;
unsigned long lastMillis = 0;

void loop() {
	if (pb.update() && pb.read()) {
		pwm += 10;
		if (pwm > 100) {
			pwm = 0;
		}
		analogWrite(OE_PIN, 255 - pwm);
		Serial.println(String("PWM ") + pwm);
	}
	auto currentMillis = millis();
	if (currentMillis - lastMillis < 1000) {
		return;
	}
	lastMillis = currentMillis;

	auto dateTime = rtc.GetDateTime();
	printDateTime(dateTime);

	//else {
	//	return;
	//}

	//Serial.println(i);

	//for (int j = 0; j < 8; j++) {
	//	digitalWrite(CLOCK_PIN, LOW);
	//	digitalWrite(DATA_PIN, (8 - i) == j);
	//	digitalWrite(CLOCK_PIN, HIGH);
	//	digitalWrite(DATA_PIN, LOW);
	//}

	//for (int j = 0; j < 8; j++) {
	//	digitalWrite(CLOCK_PIN, LOW);
	//	digitalWrite(DATA_PIN, (8 - i) == j);
	//	digitalWrite(CLOCK_PIN, HIGH);
	//	digitalWrite(DATA_PIN, LOW);
	//}

	//for (int j = 0; j < 8; j++) {
	//	digitalWrite(CLOCK_PIN, LOW);
	//	digitalWrite(DATA_PIN, (8 - i) == j);
	//	digitalWrite(CLOCK_PIN, HIGH);
	//	digitalWrite(DATA_PIN, LOW);
	//}

	//for (int j = 0; j < 8; j++) {
	//	digitalWrite(CLOCK_PIN, LOW);
	//	digitalWrite(DATA_PIN, (8 - i) == j);
	//	digitalWrite(CLOCK_PIN, HIGH);
	//	digitalWrite(DATA_PIN, LOW);
	//}

	//shiftOut(getDigit(i));
	//shiftOut(getDigit(i));
	//shiftOut(getDigit(i));
	//shiftOut(getDigit(i));

	digitalWrite(CLOCK_PIN, LOW);
	digitalWrite(LATCH_PIN, LOW);


	//shiftOut(getDigit(i));
	//shiftOut(getDigit(i));
	//shiftOut(getDigit(i));
	//shiftOut(getDigit(i));

	shiftOut(getDigit(dateTime.Second() % 10));
	shiftOut(getDigit(dateTime.Second() / 10));
	shiftOut(getDigit(dateTime.Minute() % 10));
	shiftOut(getDigit(dateTime.Minute() / 10));

	//shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, ~uint8_t(1 << i));
	//shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, ~uint8_t(1 << i));
	//shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, ~uint8_t(1 << i));
	//shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, ~uint8_t(1 << i));
	/*
	for (int j = 0; j < 32; j++) {
		bool high = (31 - i) != j;

		digitalWrite(DATA_PIN, high);
		sleep();
		digitalWrite(CLOCK_PIN, HIGH);
		sleep();
		digitalWrite(DATA_PIN, LOW);
		sleep();
		digitalWrite(CLOCK_PIN, LOW);
		sleep();
	}
	*/
	delay(2);
	digitalWrite(LATCH_PIN, HIGH);
	delay(2);
	sleep();

	if (++i >= 10) {
		i = 0;
	}

	return;

	for (int j = 0; j < numOfRegisterPins; j++) {
		setRegisterPin(j, i != j);
	}
	if (++i >= numOfRegisterPins) {
		i = 0;
	}
	writeRegisters();
	delay(200);
}

void shiftOut(uint8_t data) {
	for (int i = 7; i >= 0; i--) {
		digitalWrite(CLOCK_PIN, LOW);
		sleep();
		digitalWrite(DATA_PIN, (data & (1 << i)) == 0);
		sleep();
		digitalWrite(CLOCK_PIN, HIGH);
		sleep();
		digitalWrite(DATA_PIN, LOW);
		sleep();
	}
}

#define BUILD_DIGIT(_x1, _1, _x2, _6, _x3, _2, _x4, _7, _x5, _5, _x6, _3, _x7, _4, _x8) \
( \
	(!!(_1) << 0) | \
	(!!(_2) << 1) | \
	(!!(_3) << 2) | \
	(!!(_4) << 3) | \
	(!!(_5) << 4) | \
	(!!(_6) << 5) | \
	(!!(_7) << 6) \
)

int getDigit(int digit) {
	switch (digit) {
	case 0:
		return BUILD_DIGIT(
			, 1, ,
			1, , 1,
			, 0, ,
			1, , 1,
			, 1,
			);
	case 1:
		return BUILD_DIGIT(
			 , 0,  ,
			0,  , 1,
			 , 0,  ,
			0,  , 1,
			 , 0,  
		);
	case 2:
		return BUILD_DIGIT(
			, 1, ,
			0, , 1,
			, 1, ,
			1, , 0,
			, 1,
		);
	case 3:
		return BUILD_DIGIT(
			, 1, ,
			0, , 1,
			, 1, ,
			0, , 1,
			, 1,
		);
	case 4:
		return BUILD_DIGIT(
			, 0, ,
			1, , 1,
			, 1, ,
			0, , 1,
			, 0,
		);
	case 5:
		return BUILD_DIGIT(
			, 1, ,
			1, , 0,
			, 1, ,
			0, , 1,
			, 1,
		);
	case 6:
		return BUILD_DIGIT(
			, 1, ,
			1, , 0,
			, 1, ,
			1, , 1,
			, 1,
		);
	case 7:
		return BUILD_DIGIT(
			, 1, ,
			0, , 1,
			, 0, ,
			0, , 1,
			, 0,
		);
	case 8:
		return BUILD_DIGIT(
			, 1, ,
			1, , 1,
			, 1, ,
			1, , 1,
			, 1,
		);
	case 9:
		return BUILD_DIGIT(
			, 1, ,
			1, , 1,
			, 1, ,
			0, , 1,
			, 1,
		);
	default:
		return 0;
	}
}

void clearRegisters() {
	// Reset all register pins
	for (int i = numOfRegisterPins - 1; i >= 0; i--) {
		registers[i] = LOW;
	}
}

void setRegisterPin(int index, int value) {
	// Set an individual pin HIGH or LOW
	registers[index] = value;
}

void writeRegisters() {
	// Set and display registers
	digitalWrite(CLOCK_PIN, LOW);
	digitalWrite(LATCH_PIN, LOW);
	sleep();

	for (int i = numOfRegisterPins - 1; i >= 0; i--) {
		digitalWrite(CLOCK_PIN, LOW);
		sleep();
		digitalWrite(DATA_PIN, registers[i]);
		sleep();
		digitalWrite(CLOCK_PIN, HIGH);
		sleep();
		digitalWrite(DATA_PIN, LOW);
	}

	digitalWrite(LATCH_PIN, HIGH);
	sleep();
}


volatile int sleepCounter = 0;

void sleep() {
	for (int i = 0; i < 1; i++) {
		sleepCounter++;
	}
}
/*

//Pin connected to ST_CP of 74HC595
int latchPin = 4;
//Pin connected to SH_CP of 74HC595
int clockPin = 3;
////Pin connected to DS of 74HC595
int dataPin = 2;

int oePin = 5;

void shiftOut(int myDataPin, int myClockPin, byte myDataOut);

void setup() {
	//Start Serial for debugging purposes
	Serial.begin(9600);
	//set pins to output because they are addressed in the main loop
	pinMode(latchPin, OUTPUT);
	pinMode(clockPin, OUTPUT);
	pinMode(dataPin, OUTPUT);

	// Pull OE to ground
	pinMode(oePin, OUTPUT);
	digitalWrite(oePin, LOW);
}
void loop() {
	//count up routine
	for (int j = 0; j < 8; j++) {
		//ground latchPin and hold low for as long as you are transmitting
		digitalWrite(latchPin, 0);

		shiftOut(dataPin, clockPin, MSBFIRST, ~byte(1 << j));
		shiftOut(dataPin, clockPin, MSBFIRST, ~byte(1 << (j - 8)));
		shiftOut(dataPin, clockPin, MSBFIRST, ~byte(1 << (j - 16)));
		shiftOut(dataPin, clockPin, MSBFIRST, ~byte(1 << (j - 24)));

		//shiftOut(dataPin, clockPin, byte(1 << j));
		//shiftOut(dataPin, clockPin, byte(1 << (j - 8)));
		//shiftOut(dataPin, clockPin, byte(1 << (j - 16)));
		//shiftOut(dataPin, clockPin, byte(1 << (j - 24)));

		//return the latch pin high to signal chip that it
		//no longer needs to listen for information
		digitalWrite(latchPin, 1);
		delay(200);
	}
}
*/

