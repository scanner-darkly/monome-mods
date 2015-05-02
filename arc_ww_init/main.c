#include <stdio.h>

// asf
#include "delay.h"
#include "compiler.h"
#include "flashc.h"
#include "preprocessor.h"
#include "print_funcs.h"
#include "intc.h"
#include "pm.h"
#include "gpio.h"
#include "spi.h"
#include "sysclk.h"

// skeleton
#include "types.h"
#include "events.h"
#include "init.h"
#include "interrupts.h"
#include "monome.h"
#include "timers.h"
#include "adc.h"
#include "util.h"
#include "ftdi.h"

// this
#include "conf_board.h"

	
#define FIRSTRUN_KEY 0x22 // flag to indicate if flash is fresh
#define ENCODER_DELTA_SENSITIVITY 40 // to reduce jitter on encoders

// scales - use these values for CV A and CV B, there are copied from the white whale
const u16 SCALES[24][16] = {

0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,													// ZERO
0, 68, 136, 170, 238, 306, 375, 409, 477, 545, 579, 647, 715, 784, 818,	886,					// ionian [2, 2, 1, 2, 2, 2, 1]
0, 68, 102, 170, 238, 306, 340, 409, 477, 511, 579, 647, 715, 750, 818,	886,					// dorian [2, 1, 2, 2, 2, 1, 2]
0, 34, 102, 170, 238, 272, 340, 409, 443, 511, 579, 647, 681, 750, 818,	852,					// phrygian [1, 2, 2, 2, 1, 2, 2]
0, 68, 136, 204, 238, 306, 375, 409, 477, 545, 613, 647, 715, 784, 818,	886,					// lydian [2, 2, 2, 1, 2, 2, 1]
0, 68, 136, 170, 238, 306, 340, 409, 477, 545, 579, 647, 715, 750, 818,	886,					// mixolydian [2, 2, 1, 2, 2, 1, 2]
0, 68, 136, 170, 238, 306, 340, 409, 477, 545, 579, 647, 715, 750, 818,	886,					// aeolian [2, 1, 2, 2, 1, 2, 2]
0, 34, 102, 170, 204, 272, 340, 409, 443, 511, 579, 613, 681, 750, 818,	852,					// locrian [1, 2, 2, 1, 2, 2, 2]

0, 34, 68, 102, 136, 170, 204, 238, 272, 306, 341, 375, 409, 443, 477, 511,						// chromatic
0, 68, 136, 204, 272, 341, 409, 477, 545, 613, 682, 750, 818, 886, 954,	1022,					// whole
0, 170, 340, 511, 681, 852, 1022, 1193, 1363, 1534, 1704, 1875, 2045, 2215, 2386, 2556,			// fourths
0, 238, 477, 715, 954, 1193, 1431, 1670, 1909, 2147, 2386, 2625, 2863, 3102, 3340, 3579,		// fifths
0, 17, 34, 51, 68, 85, 102, 119, 136, 153, 170, 187, 204, 221, 238,	255,						// quarter
0, 8, 17, 25, 34, 42, 51, 59, 68, 76, 85, 93, 102, 110, 119, 127,								// eighth
0, 61, 122, 163, 245, 327, 429, 491, 552, 613, 654, 736, 818, 920, 982, 1105, 					// just
0, 61, 130, 163, 245, 337, 441, 491, 552, 621, 654, 736, 828, 932, 982, 1105,					// pythagorean

0, 272, 545, 818, 1090, 1363, 1636, 1909, 2181, 2454, 2727, 3000, 3272, 3545, 3818, 4091,		// equal 10v
0, 136, 272, 409, 545, 682, 818, 955, 1091, 1228, 1364, 1501, 1637, 1774, 1910, 2047,			// equal 5v
0, 68, 136, 204, 272, 341, 409, 477, 545, 613, 682, 750, 818, 886, 954, 1023,					// equal 2.5v
0, 34, 68, 102, 136, 170, 204, 238, 272, 306, 340, 374, 408, 442, 476, 511,						// equal 1.25v
0, 53, 118, 196, 291, 405, 542, 708, 908, 1149, 1441, 1792, 2216, 2728, 3345, 4091,				// log-ish 10v
0, 136, 272, 409, 545, 682, 818, 955, 1091, 1228, 1364, 1501, 1637, 1774, 1910, 2047,			// log-ish 5v
0, 745, 1362, 1874, 2298, 2649, 2941, 3182, 3382, 3548, 3685, 3799, 3894, 3972, 4037, 4091,		// exp-ish 10v
0, 372, 681, 937, 1150, 1325, 1471, 1592, 1692, 1775, 1844, 1901, 1948, 1987, 2020, 2047		// exp-ish 5v

};

u16 adc[4]; // used to store values read from ADC
u8 front_timer; // used to detect long press on the front panel button

// variables used by the internal clock implementation
u8 clock_phase;
u16 clock_time, clock_temp;

// used to update CV A and CV B outputs
u16 cv0, cv1;

u16 encoderDelta[4] = {0, 0, 0, 0}; // used to accumulate encoder deltas
u8 prevPotValue = 0xffff; // used to detect if the module knob was turned, setting initial value to outside of pot range (0-4095)

u8 values[4] = {0, 0, 0, 0}; // values to be controlled by the encoders - replace with yours

// structure for whatever you want to store in flash memory
typedef const struct {
	u8 fresh;
	// declare whatever variables you want to store here
} nvram_data_t;
static nvram_data_t flashy;

// NVRAM data structure located in the flash array.
__attribute__((__section__(".flash_nvram")))
static nvram_data_t flashy;


////////////////////////////////////////////////////////////////////////////////
// prototypes

static void clock(u8 phase);

// start/stop monome polling/refresh timers
extern void timers_set_monome(void);
extern void timers_unset_monome(void);

// check the event queue
static void check_events(void);

// handler protos
static void handler_None(s32 data) { ;; }
static void handler_KeyTimer(s32 data);
static void handler_Front(s32 data);
static void handler_ClockNormal(s32 data);

u8 flash_is_fresh(void);
void flash_write(void);
void flash_read(void);

void updateCvOuts(void);
void updateArc(void);


////////////////////////////////////////////////////////////////////////////////
// application clock code

void updateCvOuts(void)
{
	// let's output values[0] to CV A and values[1] to CV B
	// output range is 0..4095, values[] is 0..64, so we need to scale
	
	cv0 = values[0] << 6; if (cv0 > 4095) cv0 = 4095; // keep it within the range
	cv1 = values[1] << 6; if (cv1 > 4095) cv1 = 4095; 

	// write to DAC
	spi_selectChip(SPI,DAC_SPI);
	spi_write(SPI,0x31);	// update A
	spi_write(SPI,cv0>>4);
	spi_write(SPI,cv0<<4);
	spi_unselectChip(SPI,DAC_SPI);

	spi_selectChip(SPI,DAC_SPI);
	spi_write(SPI,0x38);	// update B
	spi_write(SPI,cv1>>4);
	spi_write(SPI,cv1<<4);
	spi_unselectChip(SPI,DAC_SPI);
}

void updateArc(void)
{
	// to update LEDs use monomeLedBuffer[]
	// there are 64 LEDs in each ring, so to get the actual LED index multiply the ring # by 64 and add the LED #
	// each LED can bet set to values 0..15 (for varibright) 0 being completely off and 15 being the brightest
	
	// monome_encs() will give you the total number of encoders the connected arc has
	for (u8 enc = 0; enc < monome_encs(); enc++)
	{
		for (u8 led = 0; led < 64; led++)
		{
			// this will light all LEDs that have index smaller than the value for that encoder
			// essentially displaying the value
			monomeLedBuffer[(enc << 6) + led] = led < values[enc] ? 15 : 0; 
		}
	}

	// now you need to update monomeFrameDirty so that the arc LEDs get updated after we update monomeLedBuffer[]
	// each bit corresponds to a ring - if any LEDs changed in that ring you want to set the corresponding bit to 1
	// here we simply tell it to update all 4 rings
	monomeFrameDirty = 0b1111;
}

// this gets called whenever there is a clock trigger
void clock(u8 phase) {
	if(phase) {
		gpio_set_gpio_pin(B10); // set the clock output
		
		// here you would do whatever should happen on each clock trigger
		// like advancing a step in a sequencer
		// don't to anything too long here - consider using timers for that
	}
	else {
		gpio_clr_gpio_pin(B10); // clear the clock output
 	}
}


////////////////////////////////////////////////////////////////////////////////
// timers

static softTimer_t clockTimer = { .next = NULL, .prev = NULL };
static softTimer_t keyTimer = { .next = NULL, .prev = NULL };
static softTimer_t adcTimer = { .next = NULL, .prev = NULL };
static softTimer_t monomePollTimer = { .next = NULL, .prev = NULL };
static softTimer_t monomeRefreshTimer  = { .next = NULL, .prev = NULL };
static softTimer_t showValueTimer = { .next = NULL, .prev = NULL };

static void clockTimer_callback(void* o) {  
	if(clock_external == 0) {
		clock_phase++;
		if(clock_phase>1) clock_phase=0;
		(*clock_pulse)(clock_phase);
	}
}

static void keyTimer_callback(void* o) {  
	static event_t e;
	e.type = kEventKeyTimer;
	e.data = 0;
	event_post(&e);
}

static void adcTimer_callback(void* o) {  
	static event_t e;
	e.type = kEventPollADC;
	e.data = 0;
	event_post(&e);
}

// monome polling callback
static void monome_poll_timer_callback(void* obj) {
  // asynchronous, non-blocking read
  // UHC callback spawns appropriate events
	ftdi_read();
}

// monome refresh callback
static void monome_refresh_timer_callback(void* obj) {
	if(monomeFrameDirty > 0) {
		static event_t e;
		e.type = kEventMonomeRefresh;
		event_post(&e);
	}
}

// monome: start polling
void timers_set_monome(void) {
	timer_add(&monomePollTimer, 20, &monome_poll_timer_callback, NULL );
	timer_add(&monomeRefreshTimer, 30, &monome_refresh_timer_callback, NULL );
}

// monome stop polling
void timers_unset_monome(void) {
	timer_remove( &monomePollTimer );
	timer_remove( &monomeRefreshTimer ); 
}


////////////////////////////////////////////////////////////////////////////////
// event handlers

static void handler_FtdiConnect(s32 data) { ftdi_setup(); }
static void handler_FtdiDisconnect(s32 data) { 
	timers_unset_monome();
	// event_t e = { .type = kEventMonomeDisconnect };
	// event_post(&e);
}

// handler tracking when arc is connected
static void handler_MonomeConnect(s32 data) {
	// start timers that refresh arc and track arc events
	timers_set_monome();
}

// handler that takes care of polling for arc events
static void handler_MonomePoll(s32 data) { monome_read_serial(); }

// handler that takes care of tracking when arc needs to be refreshed
static void handler_MonomeRefresh(s32 data) {
	if(monomeFrameDirty) {
		(*monome_refresh)();
	}
}

// handler for the front panel button pressed event
static void handler_Front(s32 data) {
	if(data == 0) {
		// front button pressed
		front_timer = 15; // start tracking long press
		
		// do whatever you need to do when the front button pressed here
		// don't do anything too long - for long operations consider using timers
	}
	else {
		front_timer = 0;
	}
}

// handler for the ADC event
static void handler_PollADC(s32 data) {
	u16 i;
	adc_convert(&adc);

	// CLOCK POT INPUT
	i = adc[0];
	i = i>>2;
	if(i != clock_temp) {
		// 1000ms - 24ms
		clock_time = 25000 / (i + 25);
		timer_set(&clockTimer, clock_time);
	}
	clock_temp = i;

	// PARAM POT INPUT
	// value will be in the 0-4095 range, scale accordingly
	// also be aware there is quite a bit of noise on the input, so you might want to desensitize it a bit
	// by averaging several values or some other technique
	u8 newPotValue = adc[1]; 
	
	if (newPotValue != prevPotValue)
	{
		prevPotValue = newPotValue;
		// value changed which means the param knob was turned, do whatever you need to do here
		// don't do anything too long - for long operations consider using timers
	}
}

// handler for the kEventSaveFlash event
static void handler_SaveFlash(s32 data) {
	flash_write();
}

// handler to detect front panel button long press
static void handler_KeyTimer(s32 data) {
	if(front_timer) {
		if(front_timer == 1) {
			static event_t e;
			// long press happened - here it fires up the event for SaveFlash
			e.type = kEventSaveFlash;
			event_post(&e);
			front_timer--;
		}
		else front_timer--;
	}
}

static void handler_ClockNormal(s32 data) {
	clock_external = !gpio_get_pin_value(B09); 
}

// handler for arc encoders event, gets called whenever an encoder is turned
static void handler_MonomeRingEnc(s32 data) { 
	u8 n;
	s8 delta;
	monome_ring_enc_parse_event_data(data, &n, &delta);
	// n will be the encoder # 0..3
	// delta is a signed value which indicates how much the encoder was turned
	// clockwise movement will generate positive delta, CCW - negative
	
	// i find that for simple tasks you might want to "desensitize" the input a bit
	// the following block achieves that by accumulating delta and only reacting to it when it passes the threshold
	// you might want to adjust or change this for your specific needs
	encoderDelta[n] += abs(delta);
	if (encoderDelta[n] < ENCODER_DELTA_SENSITIVITY)
		return;
	encoderDelta[n] = 0;

	// example on processing encoder events - here we update values array accordingly and refresh the arc
	// don't do anything too long here - for long operations consider using timers
	switch(n)
	{
		case 0: // first encoder turned
			if (delta > 0) // turned clockwise
			{
				if (values[0] < 64) values[0]++;
			}
			else // turned counter clockwise
			{
				if (values[0] > 0) values[0]--;
			}
			break;
		case 1: // second encoder turned
			if (delta > 0) // turned clockwise
			{
				if (values[1] < 64) values[1]++;
			}
			else // turned counter clockwise
			{
				if (values[1] > 0) values[1]--;
			}
			break;
		case 2: // third encoder turned
			if (delta > 0) // turned clockwise
			{
				if (values[2] < 64) values[2]++;
			}
			else // turned counter clockwise
			{
				if (values[2] > 0) values[2]--;
			}
			break;
		case 3: // fourth encoder turned
			if (delta > 0) // turned clockwise
			{
				if (values[3] < 64) values[3]++;
			}
			else // turned counter clockwise
			{
				if (values[3] > 0) values[3]--;
			}
			break;
	}
	updateArc();
	updateCvOuts();
}


////////////////////////////////////////////////////////////////////////////////
// 

// assign event handlers
static inline void assign_main_event_handlers(void) {
	app_event_handlers[ kEventFront ]	= &handler_Front;
	// app_event_handlers[ kEventTimer ]	= &handler_Timer;
	app_event_handlers[ kEventPollADC ]	= &handler_PollADC;
	app_event_handlers[ kEventKeyTimer ] = &handler_KeyTimer;
	app_event_handlers[ kEventSaveFlash ] = &handler_SaveFlash;
	app_event_handlers[ kEventClockNormal ] = &handler_ClockNormal;
	app_event_handlers[ kEventFtdiConnect ]	= &handler_FtdiConnect ;
	app_event_handlers[ kEventFtdiDisconnect ]	= &handler_FtdiDisconnect ;
	app_event_handlers[ kEventMonomeConnect ]	= &handler_MonomeConnect ;
	app_event_handlers[ kEventMonomeDisconnect ]	= &handler_None ;
	app_event_handlers[ kEventMonomePoll ]	= &handler_MonomePoll ;
	app_event_handlers[ kEventMonomeRefresh ]	= &handler_MonomeRefresh ;
	app_event_handlers[ kEventMonomeRingEnc ]	= &handler_MonomeRingEnc ;
}

// app event loop
void check_events(void) {
	static event_t e;
	if( event_next(&e) ) {
		(app_event_handlers)[e.type](e.data);
	}
}

// checks if flash was previously written to
u8 flash_is_fresh(void) {
	return (flashy.fresh != FIRSTRUN_KEY);
}

// write to flash
void flash_write(void) {
	// set "fresh" status so we know something was stored in flash previously
	flashc_memset8((void*)&(flashy.fresh), FIRSTRUN_KEY, 1, true);
	// update flash with your variables:
	// make sure to modify nvram_data_t structure to include whatever you want to store
	// replace # with number of bytes you're storing and use the appropriate flashc_memset# method
	// flashc_memset8((void*)&(flashy.myVar), myVar, #, true);
}

// read from flash
void flash_read(void) {
	// initialize your variables from flash here like this 
	// (make sure to modify nvram_data_t structure to include whatever you want to store)
	// myVar = flashy.myVar;
}


////////////////////////////////////////////////////////////////////////////////
// main

int main(void)
{
	sysclk_init();

	init_dbg_rs232(FMCK_HZ);

	init_gpio();
	assign_main_event_handlers();
	init_events();
	init_tc();
	init_spi();
	init_adc();

	irq_initialize_vectors();
	register_interrupts();
	cpu_irq_enable();

	init_usb_host();
	init_monome();

	if(flash_is_fresh()) {
		// nothing has been stored in the flash memory so far
		// so you need to initialize any variables you store in flash here with appropriate default values
	}
	else {
		// read from flash
		flash_read();
	}

	clock_pulse = &clock;
	clock_external = !gpio_get_pin_value(B09);

	// start timers that track clock, ADCs (including the clock and the param knobs) and the front panel button
	timer_add(&clockTimer,120,&clockTimer_callback, NULL);
	timer_add(&keyTimer,50,&keyTimer_callback, NULL);
	timer_add(&adcTimer,100,&adcTimer_callback, NULL);
	clock_temp = 10000; // out of ADC range to force tempo

	// main loop - you probably don't need to do anything here as everything should be done by handlers
	while (true) {
		check_events();
	}
}
