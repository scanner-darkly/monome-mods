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

	
#define FIRSTRUN_KEY 0x22
#define ENCODER_DELTA_SENSITIVITY 40

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

const u8 DIVISORS[16][4] =
{
	1, 2, 3, 4,
	1, 2, 4, 8,
	2, 4, 8, 16,
	2, 3, 4, 5,

	2, 3, 5, 7,
	2, 3, 5, 8,
	3, 5, 7, 9,
	3, 6, 9, 12,

	12, 9, 6, 3,
	9, 7, 5, 3,
	8, 5, 3, 2,
	7, 5, 3, 2,

	5, 4, 3, 2,
	16, 8, 4, 2,
	8, 4, 2, 1,
	4, 3, 2, 1
};

const u8 MIXERS[16][2] =
{
	0b1111, 0b0001,
	0b1110, 0b0001,
	0b1100, 0b0011,
	0b1000, 0b0111,
	
	0b1010, 0b0101,
	0b0101, 0b1010,
	0b1001, 0b0110,
	0b0110, 0b1001,
	
	0b1111, 0b0011,
	0b1111, 0b0110,
	0b1111, 0b1100,
	0b1111, 0b1000,
	
	0b0001, 0b1110,
	0b0011, 0b1100,
	0b1000, 0b1110,
	0b0001, 0b1111
};

const u8 TRIGGERS[4] = {B00, B01, B02, B03};

u16 adc[4];
u8 front_timer;

u8 clock_phase;
u16 clock_time, clock_temp;

u16 cv0, cv1;

u8 scale, phase, divisor, chance, mixer;
u16 counter[4] = {0, 0, 0, 0};
u8 prevValue[4];
static softTimer_t triggerTimer = { .next = NULL, .prev = NULL };

u16 encoderDelta[4] = {0, 0, 0, 0};
u8 valueToShow = 0;
u8 prevPotValue = 16;
u8 arc2index = 0; // 0 - show rings 1&2, 1 - show rings 3&4

typedef const struct {
	u8 fresh;
	u8 phase, divisor, chance, mixer;
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
void flash_unfresh(void);
void flash_write(void);
void flash_read(void);

void updateArc(bool updateTriggers, bool resetTriggers);
void showValue(u8 value, bool updateTriggers, bool resetTriggers);
static void triggerTimer_callback(void* o);

////////////////////////////////////////////////////////////////////////////////
// application clock code

void updateArc(bool updateTriggers, bool resetTriggers)
{
	u8 cv = 0, cv_ = 0, realLed, prevLed, prev, next, level;
	
	if (updateTriggers) timer_remove(&triggerTimer);
	
	for (u8 enc = 0; enc < 4; enc++)
	{
		prev = 0;
		
		for (u8 led = 0; led < 64; led++)
		{
			// adding 192 = 64*3 to cover negative values (phase can go up to 32 and enc is 4 max plus another 64 to cover -led)
			u16 factor = (counter[enc] + 192 - led - phase*enc) / DIVISORS[divisor][enc]; 
			realLed = (counter[enc] + 64 - led + 32) & 63;
			next = factor & 1;
			if (!prev && next)
			{
				level = 10 - (abs(led - 32) >> 2);
			}
			prev = next;
			monomeLedBuffer[(enc << 6) + realLed] = next * level;
		}

		realLed = (enc << 6) + (counter[enc] & 63);

		if (resetTriggers)
		{
			prevLed = (enc << 6) + ((counter[enc] + 63) & 63);
			prevValue[enc] = monomeLedBuffer[prevLed];
		}

		if (chance < (rnd() & 63))
		{
			if (monomeLedBuffer[realLed] && (MIXERS[mixer][0] & (1 << enc))) cv += 1 << enc;
			if (updateTriggers && prevValue[enc] != monomeLedBuffer[realLed]) gpio_set_gpio_pin(TRIGGERS[enc]); else gpio_clr_gpio_pin(TRIGGERS[enc]);
		}
		
		if (monomeLedBuffer[realLed] && (MIXERS[mixer][1] & (1 << enc))) cv_ += 1 << enc;
		
		prevValue[enc] = monomeLedBuffer[realLed];
		monomeLedBuffer[realLed] = 0xF;
	}
	timer_add(&triggerTimer, 10, &triggerTimer_callback, NULL);

	cv0 = SCALES[scale + 1][cv];
	cv1 = SCALES[scale + 1][cv_];

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
	
	if (monome_encs() == 2) // arc2
	{
		if (valueToShow == 1) // phase 0-32
		{
			for (u8 led = 0; led < 64; led++)
				monomeLedBuffer[63 - led] = led < (phase << 1) ? 12 : 0;
		}
		if (valueToShow == 2) // divisor 0-15
		{
			for (u8 led = 0; led < 64; led++)
				monomeLedBuffer[led] = (led < ((divisor + 1) << 2)) && (led >= (divisor << 2)) ? 12 : 0;
		}
		if (valueToShow == 3) // chance 0-32
		{
			for (u8 led = 0; led < 64; led++)
				monomeLedBuffer[127 - led] = led < (chance << 1) ? 12 : 0;
		}
		if (valueToShow == 4) // mixer 0-15
		{
			for (u8 led = 0; led < 64; led++)
				monomeLedBuffer[led + 64] = (led < ((mixer + 1) << 2)) && (led >= (mixer << 2)) ? 12 : 0;
		}
		if (valueToShow == 5)
		{
			for (u8 led = 0; led < 64; led++)
				monomeLedBuffer[led] = (led < ((scale + 1) << 2)) && (led >= (scale << 2)) ? 12 : 0;
		}
		if (valueToShow == 0)
		{
			if (arc2index)
			{
				for (u8 led = 0; led < 64; led++)
				{
					monomeLedBuffer[led] = monomeLedBuffer[led + 128];
					monomeLedBuffer[led + 64] = monomeLedBuffer[led + 192];
				}
			}
		}
		// just so i can test on arc4
		for (u8 led = 0; led < 64; led++)
		{
			monomeLedBuffer[led + 128] = monomeLedBuffer[led + 192] = 0;
		}
	}
	else
	{
		if (valueToShow == 1 || valueToShow == 6) // phase 0-32
		{
			for (u8 led = 0; led < 64; led++)
				monomeLedBuffer[led] = led < (phase << 1) ? 12 : 0;
		}
		if (valueToShow == 2 || valueToShow == 6) // divisor 0-15
		{
			for (u8 led = 0; led < 64; led++)
				monomeLedBuffer[led + 64] = (led < ((divisor + 1) << 2)) && (led >= (divisor << 2)) ? 12 : 0;
		}
		if (valueToShow == 3 || valueToShow == 6) // chance 0-32
		{
			for (u8 led = 0; led < 64; led++)
				monomeLedBuffer[led + 128] = led < (chance << 1) ? 12 : 0;
		}
		if (valueToShow == 4 || valueToShow == 6) // mixer 0-15
		{
			for (u8 led = 0; led < 64; led++)
				monomeLedBuffer[led + 192] = (led < ((mixer + 1) << 2)) && (led >= (mixer << 2)) ? 12 : 0;
		}
		if (valueToShow == 5)
		{
			for (u8 led = 0; led < 64; led++)
				monomeLedBuffer[led] = (led < ((scale + 1) << 2)) && (led >= (scale << 2)) ? 12 : 0;
		}
	}

	monomeFrameDirty = 0b1111;
}

//---

void clock(u8 phase) {
	if(phase) {
		gpio_set_gpio_pin(B10);
		
		for (u8 enc = 0; enc < 4; enc++)
		{
			counter[enc]++;
			if (counter[enc] >= ((u16)DIVISORS[divisor][enc] << 6)) counter[enc] = 0;
		}
		updateArc(true, false);
	}
	else {
		gpio_clr_gpio_pin(B10);
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

static void showValueTimer_callback(void* o) {  
	valueToShow = 0;
	timer_remove(&showValueTimer);
	updateArc(false, false);
}

void showValue(u8 value, bool updateTriggers, bool resetTriggers)
{
	if (valueToShow)
		timer_remove(&showValueTimer);

	valueToShow = value;
	timer_add(&showValueTimer, 1000, &showValueTimer_callback, NULL);
	updateArc(updateTriggers, resetTriggers);
}

static void triggerTimer_callback(void* o) {  
	timer_remove(&triggerTimer);
	gpio_clr_gpio_pin(B00);
	gpio_clr_gpio_pin(B01);
	gpio_clr_gpio_pin(B02);
	gpio_clr_gpio_pin(B03);
}

////////////////////////////////////////////////////////////////////////////////
// event handlers

static void handler_FtdiConnect(s32 data) { ftdi_setup(); }
static void handler_FtdiDisconnect(s32 data) { 
	timers_unset_monome();
	// event_t e = { .type = kEventMonomeDisconnect };
	// event_post(&e);
}

static void handler_MonomeConnect(s32 data) {
	timers_set_monome();
}

static void handler_MonomePoll(s32 data) { monome_read_serial(); }
static void handler_MonomeRefresh(s32 data) {
	if(monomeFrameDirty) {
		(*monome_refresh)();
	}
}

static void handler_Front(s32 data) {
	if(data == 0) {
		front_timer = 15;
		
		if (monome_encs() == 2) // arc2
		{
			arc2index = !arc2index;
			updateArc(false, false);
		}
		else
		{
			showValue(6, false, false);
		}
	}
	else {
		front_timer = 0;
	}
}

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
	u8 newPotValue = adc[1] >> 8; // should be in the range 0-15
	if (newPotValue != prevPotValue)
	{
		prevPotValue = scale = newPotValue;
		showValue(5, false, false);
	}
}

static void handler_SaveFlash(s32 data) {
	flash_write();
}

static void handler_KeyTimer(s32 data) {
	if(front_timer) {
		if(front_timer == 1) {
			static event_t e;
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


////////////////////////////////////////////////////////////////////////////////
// application arc code

static void handler_MonomeRingEnc(s32 data) { 
	u8 n;
	s8 delta;
	monome_ring_enc_parse_event_data(data, &n, &delta);
	
	encoderDelta[n] += abs(delta);
	if (encoderDelta[n] < ENCODER_DELTA_SENSITIVITY)
		return;
	
	encoderDelta[n] = 0;
	
	if (monome_encs() == 2) // arc2
	{
		switch(n)
		{
			case 0:
				if (delta < 0)
				{
					if (++phase > 32) phase = 0; 
					showValue(1, false, true);
				} 
				else 
				{ 
					if (++divisor > 15) divisor = 0;
					for (u8 enc = 0; enc < 4; enc++) counter[enc] = 0;
					showValue(2, false, true);
				}
				break;
			case 1:
				if (delta < 0)
				{
					if (++chance > 32) chance = 0; 
					showValue(3, false, false);
				}
				else
				{
					if (++mixer > 15) mixer = 0;
					showValue(4, false, false);
				}
				break;
		}
	}
	else
	{
		switch(n)
		{
			case 0:
				if (delta > 0)
				{
					if (phase < 32) phase++; 
				} 
				else 
				{ 
					if (phase > 0) phase--; 
				}
				showValue(1, false, true);
				break;
			case 1:
				if (delta > 0)
				{
					if (++divisor > 15) divisor = 0;
				}
				else
				{
					if (divisor > 0) divisor--; else divisor = 15;
				}
				for (u8 enc = 0; enc < monome_encs(); enc++) counter[enc] = 0;
				showValue(2, false, true);
				break;
			case 2:
				if (delta > 0)
				{
					if (chance < 32) chance++; 
				}
				else
				{
					if (chance > 0) chance--;
				}
				showValue(3, false, false);
				break;
			case 3:
				if (delta > 0)
				{
					if (++mixer > 15) mixer = 0;
				}
				else
				{
					if (mixer > 0) mixer--; else mixer = 15;
				}
				showValue(4, false, false);
				break;
		}
	}
}

////////////////////////////////////////////////////////////////////////////////
// application arc redraw

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

// flash commands
u8 flash_is_fresh(void) {
	return (flashy.fresh != FIRSTRUN_KEY);
}

void flash_write(void) {
	flashc_memset8((void*)&(flashy.fresh), FIRSTRUN_KEY, 4, true);
	flashc_memset8((void*)&(flashy.phase), phase, 1, true);
	flashc_memset8((void*)&(flashy.divisor), divisor, 1, true);
	flashc_memset8((void*)&(flashy.chance), chance, 1, true);
	flashc_memset8((void*)&(flashy.mixer), mixer, 1, true);
}

void flash_read(void) {
	phase = flashy.phase;
	divisor = flashy.divisor;
	chance = flashy.chance;
	mixer = flashy.mixer;
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
		phase = divisor = chance = mixer = 0;
	}
	else {
		flash_read();
	}

	clock_pulse = &clock;
	clock_external = !gpio_get_pin_value(B09);

	timer_add(&clockTimer,120,&clockTimer_callback, NULL);
	timer_add(&keyTimer,50,&keyTimer_callback, NULL);
	timer_add(&adcTimer,100,&adcTimer_callback, NULL);
	clock_temp = 10000; // out of ADC range to force tempo

	updateArc(true, true);
	
	while (true) {
		check_events();
	}
}
