/* Pin assignment
 *	PORT PIN TASK		PORT PIN TASK
 *	C6   1   Reset		C5   28  I2C
 *	D0   2   Rx		C4   27  I2C
 *	D1   3   Tx		C3   26  SPI-CS-MotionSensor
 *	D2   4			C2   25  Decoder-A2
 *	D3   5			C1   24  Decoder-A1
 *	D4   6			C0   23  Decoder-A0
 *	VCC  7			GND  22
 *	GND  8			AREF 21
 *	B6   9			AVCC 20
 *	B7   10			B5   19  SPI
 *	D5   11  OC0B-Blue	B4   18  SPI
 *	D6   12  OC0A-Green	B3   17  SPI
 *	D7   13			B2   16  SPI-CS-SDCard
 *	B0   14			B1   15  OC1A-Red
 */

#define CPU_FREQ 8000000

#define SPEED 2 //1, 2, 3, 4 or 5, changing overall speed by modifying PWM clock prescaler (1, 8, 64, 256 or 1024)

#include <stdint.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

volatile uint8_t debug;

volatile uint32_t frame = 0; //Which frame of the video is now display

volatile uint8_t led = 0; //Which of the 8 LEDs is now working

volatile float phase = 0.0f; //Current phase of the arm
volatile float angularSpeed = 0.0f; //Phase speed

volatile uint8_t imageBuffer[2][32][8][4]; //Image of current frame, double buffer, 32 phase, 8 radius, RGB (32-bit alignment), total = 1kB (2 SD card blocks)
volatile uint8_t imageBufferIndex = 0; //Double buffer: LED rendering this one, fetching SD card content and writing to the other one

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
	PORTB &= 0b11111011;
}
void spiUnselectSDCard() {
	PORTB |= 0b00000100;
}

// MCU main
int main() {

	//IO setup - All pins output or special function
	DDRB = 0xFF;
	DDRC = 0xFF;
	DDRD = 0xFF;

	//Init SPI - master
	spiUnselectMotionSensor();
	spiUnselectSDCard();
	SPCR = (1<<SPE) | (1<<MSTR) | (1<<SPR0); //Enable SPI as master, no interrupt, MSB first, SCK low on idle, f_SPI = CPU_FREQ / 16 = 0.5MHz @ 8MHz CPU
	spiMotionSensorReset();

	//Get gyroscope bias
	uint8_t gyroBias[2];
	spiMotionSensorRead(0x47, gyroBias, sizeof(gyroBias)); //Z-axis angular speed (2-byte)
	uint16_t bias = (gyroBias[0] << 8) | gyroBias[1]; //avr-gcc is little endian (LSB on lower address)

	//Setup timer 0 and 1 - RGB PWM signal
	TCCR0A = (2<<COM0A0) | (2<<COM0B0) | (3<<WGM00); //Fast PWM (update OCR at top), set at zero, clear on compare match
	TCCR1A = (2<<COM1A0) | (1<<WGM10); //Fast PWM (8-bit, update OCR at zero), set at zero, clear on compare match
	TIMSK0 = (1<<TOIE0); //ISR when overflow: Setup PWM for both timer (process next pixel)
	TCCR0B = (SPEED<<CS00); //Turn on timer
	TCCR1B = (1<<WGM12) | (SPEED<<CS10);
	/* T1 will be slightly lag after T0 because T1 is enabled after T0, but this difference will not cause any big issue */
	/* Refresh rate = 256us if scaler = 8, CPU frequency = 8MHz */

	//Setup timer 2 - Go to next frame
	TCCR2B = (7<<CS20); //Prescaller = 1024, speed = 128us, overflow (256) = 30.5Hz, @8MHz CPU
	TIMSK2 = (1<<TOIE2); //Enable overflow interrupt

	//Load default image
	for (uint8_t p = 0; p < 32; p++) {
		for (uint8_t r = 0; r < 8; r++) {
			imageBuffer[0][p][r][0] = imageBuffer[1][p][r][0] = 1 << r; //R
			imageBuffer[0][p][r][1] = imageBuffer[1][p][r][1] = 0xFF >> r; //G
			imageBuffer[0][p][r][2] = imageBuffer[1][p][r][2] = 120; //B
		}
	}
	for (uint16_t i = 0; i < 65000; i++);

	//Main process loop
	sei();
	for (;;) {
		uint8_t rawAngularSpeed[2];
		spiMotionSensorRead(0x47, rawAngularSpeed, sizeof(rawAngularSpeed)); //Z-axis angular speed (0x47H, 0x48L)
		debug = rawAngularSpeed[0];
		/* Range: +/- 250dps -->  */
		uint16_t raw = (rawAngularSpeed[0] << 8) | rawAngularSpeed[1];
		int16_t aSpeed = raw - bias; //Remove bias and offset
		float nomalizedSpeed = aSpeed / 32768.0f; //Range = [-1.0f:1.0f] = [-250dps:+250dps]
		float speedPrePeriod = nomalizedSpeed * 250.0f * 0.000256f; //Measure range = 250dps, calculate frequnecy = every 256us
		cli(); //Do not interrupt when convert and write the measured data to angular speed
		angularSpeed = speedPrePeriod;
		sei();
	}
}

// Write RGB signal to LED control
ISR (TIMER0_OVF_vect) { //Takes about 360 cycles
	/* Turn on LEDn (n=1~7), setup RGB PWM signal for LED(n+1 mod 8) */

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
	PORTC = led; //To 74LS138, one cold signal
	led = (led + 1) & 0b111;

	//Estemate current phase based on angular speed
	phase += angularSpeed;
//	while (phase > 360.1f) //Normalize phase, keep the number small to prevent overflow (think this may run for 99999 years)
//		phase -= 360.0f;
//	while (phase < 0.0f)
//		phase += 360.0f;

	uint8_t currentPhase = (uint8_t)(phase / 360.0f * 32.0f);
	currentPhase %= 32;
//	OCR1A = imageBuffer[imageBufferIndex][currentPhase][led][0];
//	OCR0A = imageBuffer[imageBufferIndex][currentPhase][led][1];
//	OCR0B = imageBuffer[imageBufferIndex][currentPhase][led][2];

	OCR0A = currentPhase * 8; //Green = current phase
	if (debug & 0x80) { //Red = angular speed if nagative direction
		OCR1A = 0 - debug;
		OCR0B = 0;
	}
	else { //Blue = angular speed if positive direction
		OCR1A = 0;
		OCR0B = debug;
	}
}

// Advance to next video frame
ISR (TIMER2_OVF_vect) {
	imageBufferIndex = (imageBufferIndex + 1) & 1;
	frame++;
	sei(); //Allow TIMER0_OVF_vect


}
