/* Pin assignment
 *	PORT PIN TASK		PORT PIN TASK
 *	C6   1   Reset		C5   28  I2C
 *	D0   2   Rx		C4   27  I2C
 *	D1   3   Tx		C3   26  SPI-CS-SDCard
 *	D2   4			C2   25  Decoder-A2
 *	D3   5			C1   24  Decoder-A1
 *	D4   6			C0   23  Decoder-A0
 *	VCC  7			GND  22
 *	GND  8			AREF 21
 *	B6   9   Crystal	AVCC 20
 *	B7   10  Crystal	B5   19  SPI
 *	D5   11  OC0B-Blue	B4   18  SPI
 *	D6   12  OC0A-Green	B3   17  SPI
 *	D7   13  MotorD		B2   16  SPI-CS-MotionSensor
 *	B0   14			B1   15  OC1A-Red
 */

#define CPU_FREQ 16000000

#define SPEED 1 //1, 2, 3, 4 or 5, changing overall speed by modifying PWM clock prescaler (1, 8, 64, 256 or 1024)

#include <stdint.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

volatile uint8_t x = 0;
volatile uint8_t motionData[6];

volatile uint32_t frame = 0; //Which frame of the video is now display
volatile uint8_t led = 0; //Which LED is now working
volatile float phase = 0.0f;

volatile uint8_t imageBuffer[2][32][8][4]; //Image of current frame, double buffer, 32 phase, 8 radius, RGB (32-bit alignment), total = 1kB (2 SD card blocks)
volatile uint8_t imageBufferIndex = 0; //LED rendering this one, fetching SD card content and writing to the other one

// MPU-6500/9250 on SPI bus
void spiSelectMotionSensor() {
	PORTC &= 0b11110111;
}
void spiUnselectMotionSensor() {
	PORTC |= 0b00001000;
}
void spiMotionSensorReset() {
	spiSelectMotionSensor(); //Reset power management
	SPDR = 0x6B | 0x00;
	while ( !(SPSR & (1<<SPIF)) );
	SPDR = 0x81;
	while ( !(SPSR & (1<<SPIF)) );
	spiUnselectMotionSensor();
	for (uint16_t i = 0; i < 20000; i++);

	spiSelectMotionSensor(); //Reset signal path
	SPDR = 0x68 | 0x00;
	while ( !(SPSR & (1<<SPIF)) );
	SPDR = 0x07;
	while ( !(SPSR & (1<<SPIF)) );
	spiUnselectMotionSensor();
	for (uint16_t i = 0; i < 20000; i++);
}
void spiMotionSensorRead(uint8_t baseAddr, uint8_t* buffer, uint8_t length) {
	spiSelectMotionSensor();
	SPDR = baseAddr | 0x80;
	while ( !(SPSR & (1<<SPIF)) );
	for (uint8_t i = 0; i < length; i++) {
		SPDR = 0xFF; //Write a dummy byte to start SPI
		while ( !(SPSR & (1<<SPIF)) );
		buffer[i] = SPDR;
	}
	spiUnselectMotionSensor();
}

// SDCard on SPI bus
void spiSelectSDCard() {
	PORTB &= 0b11111101;
}
void spiUnselectSDCard() {
	PORTB |= 0b00000010;
}

// MCU main
int main() {

	//IO setup - All pins output or special function
	DDRB = 0xFF;
	DDRC = 0xFF;
	DDRD = 0xFF;

	//Init SPI - 1MHZ master
	spiUnselectMotionSensor();
	spiUnselectSDCard();
	SPCR = (1<<SPE) | (1<<MSTR) | (1<<SPR0); //Enable SPI as master, no interrupt, MSB first, SCK low on idle, f_SPI = CPU_FREQ / 16 = 1MHz @ 16MHz CPU
	spiSelectMotionSensor();

	//Setup timer 0 - Blue and green PWM signal
	TCCR0A = (2<<COM0A0) | (2<<COM0B0) | (3<<WGM00); //Fast PWM (update OCR at top), set at zero, clear on compare match
	TCCR0B = (SPEED<<CS00); //Using system clock
	TIMSK0 = (1<<TOIE0); //ISR when overflow: Setup PWM for both timer 0 and 1 (process next pixel)

	//Setup timer 1 - Red PWM signal
	TCCR1A = (2<<COM1A0) | (1<<WGM10); //Fast PWM (8-bit, update OCR at zero), set at zero, clear on compare match
	TCCR1B = (1<<WGM12) | (SPEED<<CS10); //Using system clock

	/* T1 will be slightly lag after T0 because T1 is enabled after T0, but this difference will not cause any big issue */

	//Setup timer 2 - Go to next frame
	TCCR2B = (7<<CS20); //Prescaller = 1024, speed = 64us, overflow (256) = 61Hz, @16MHz CPU
	TIMSK2 = (1<<TOIE2); //Enable overflow interrupt

	//Setup image buffer pointer

	//Idel - ISR takes control
	sei();
//	set_sleep_mode(0);
//	sleep_mode();
	for (;;) {
//		sleep_mode();
		uint8_t mData[6];
		spiMotionSensorRead(0x3B, mData, sizeof(mData)); //ACCEL
		for (uint8_t i = 0; i < sizeof(mData); i++)
			motionData[i] = mData[i];
	}
}

// Write RGB signal to LED control
ISR (TIMER0_OVF_vect) {

	/* Note of timer OCR update:
	 * Timer 0 updates OCR at top but timer 1 does this at zero
	 * By this time (in the TIMER0_OVF_vect), timer 1 zero should passed and timer 0 top is far to arrive
	 * We can update the OCR0A, OCR0B and OCR1A register at this time, so it will be used for next pixel
	 * T0:       Zero                                                              Top           Zero
	 * T1:         Zero                                                              Top           Zero
	 * Hardware: TIMER0_OVF_vect issued                                            Update OCR0     Update OCR1
	 * Software:       Some complex computation            Write OCRnX for next pixel
	 */

	//Go to next LED, only the least 3 significant bits are used (8 LEDs)
	led = (led + 1) & 0b111;
	PORTC = led; //To 74LS138, one cold signal
	
/*	//The wing's position is between 2 pixel, we mix the color of these 2 pixels
	float currentPhase = phase;
	uint8_t lIdx = (uint8_t)currentPhase; //If at phase 20.8: lIdx = 20, weight 0.2; rIdx = 21, weight 0.8
	uint8_t rIdx = lIdx + 1;
	float lWeight = rIdx - currentPhase;
	float rWeight = currentPhase - lIdx; //NOTE: TOOOOOOOOO COMPLEX, TIME NOT ENOUGH!!!!!

	// NOTE: Do we REALLY need to use the weight average of two pixel? The average seems cannot give any advantage at all

	//Apply RGB to PWM generator (LED)
	OCR1A = image[lIdx][led][0] * lWeight + image[rIdx][led][0] * rWeight; //R
	OCR0A = image[lIdx][led][1] * lWeight + image[rIdx][led][1] * rWeight; //G
	OCR0B = image[lIdx][led][2] * lWeight + image[rIdx][led][2] * rWeight; //B
*/
	OCR1A = 128-abs(motionData[0]-128); //ACCEL_X_H
	OCR0A = 128-abs(motionData[2]-128); //ACCEL_Y_H
	OCR0B = 128-abs(motionData[4]-128); //ACCEL_Z_H
}

// Advance to next video frame
ISR (TIMER2_OVF_vect) {
	frame++;
}
