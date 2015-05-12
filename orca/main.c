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
#define DIVISOR 0
#define PHASE 1
#define RESET 2
#define CHANCE 3


u16 SCALES[16][16] = {

0, 68, 136, 170, 238, 306, 375, 409, 477, 545, 579, 647, 715, 784, 818,	886, // ionian [2, 2, 1, 2, 2, 2, 1]
0, 68, 102, 170, 238, 306, 340, 409, 477, 511, 579, 647, 715, 750, 818,	886, // dorian [2, 1, 2, 2, 2, 1, 2]
0, 34, 102, 170, 238, 272, 340, 409, 443, 511, 579, 647, 681, 750, 818,	852, // phrygian [1, 2, 2, 2, 1, 2, 2]
0, 68, 136, 204, 238, 306, 375, 409, 477, 545, 613, 647, 715, 784, 818,	886, // lydian [2, 2, 2, 1, 2, 2, 1]
0, 68, 136, 170, 238, 306, 340, 409, 477, 545, 579, 647, 715, 750, 818,	886, // mixolydian [2, 2, 1, 2, 2, 1, 2]
0, 68, 136, 170, 238, 306, 340, 409, 477, 545, 579, 647, 715, 750, 818,	886, // aeolian [2, 1, 2, 2, 1, 2, 2]
0, 34, 102, 170, 204, 272, 340, 409, 443, 511, 579, 613, 681, 750, 818,	852, // locrian [1, 2, 2, 1, 2, 2, 2]
0, 68, 102, 204, 238, 306, 340, 409, 477, 511, 613, 647, 715, 750, 818, 886, // Ukrainian Dorian

0, 34, 136, 170, 204, 272, 375, 409, 443, 545, 579, 613, 681, 784, 818, 852, // Persian
306, 375, 409, 511, 545, 579, 647, 715, 784, 818, 920, 954, 988, 1056, 1125, 1193, // Hungarian
238, 272, 375, 443, 511, 579, 613, 647, 681, 784, 852, 920, 988, 1022, 1056, 1090, // enigmatic
0, 34, 136, 170, 238, 272, 340, 409, 443, 545, 579, 647, 681, 750, 818, 852, // Phrygian dominant
0, 34, 136, 204, 272, 340, 409, 443, 545, 613, 681, 750, 818, 852, 954, 1022, // Tritone 
0, 68, 136, 204, 238, 306, 340, 409, 477, 545, 613, 647, 715, 750, 818, 886, // Acoustic 
0, 34, 102, 136, 204, 272, 340, 409, 443, 511, 545, 613, 681, 750, 818, 852,  // Altered 
0, 34, 68, 102, 136, 170, 204, 238, 272, 306, 341, 375, 409, 443, 477, 511	// chromatic

/*
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
*/

};

const u16 CHROMATIC[36] = {
	0, 34, 68, 102, 136, 170, 204, 238, 272, 306, 340, 375,
	409, 443, 477, 511, 545, 579, 613, 647, 681, 715, 750, 784,
	818, 852, 886, 920, 954, 988, 1022, 1056, 1090, 1125, 1159, 1193
};

const u8 DIVISOR_PRESETS[16][4] =
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

const u8 PHASE_PRESETS[16][4] =
{
	0, 0, 0, 0,
	0, 1, 2, 3,
	1, 2, 3, 4,
	2, 3, 4, 5,
	
	0, 2, 4, 8,
	0, 3, 6, 9,
	0, 1, 3, 5,
	0, 2, 3, 5,
	
	5, 3, 2, 0,
	5, 3, 1, 0,
	9, 6, 3, 0,
	8, 4, 2, 0,
	
	5, 4, 3, 2,
	4, 3, 2, 1,
	3, 2, 1, 0,
	1, 1, 1, 1
};

const u8 MIXER_PRESETS[16][2] =
{
	0b1110, 0b0001,
	0b1101, 0b0010,
	0b1011, 0b0100,
	0b0111, 0b1000,
	
	0b1010, 0b0101,
	0b0101, 0b1010,
	0b1001, 0b0110,
	0b0110, 0b1001,
	
	0b1110, 0b0111,
	0b0111, 0b1110,
	0b1011, 0b1101,
	0b1101, 0b1011,

	0b1111, 0b0011,
	0b1111, 0b0110,
	0b1111, 0b1100,
	0b1111, 0b1001
};

const u8 TRIGGERS[4] = {B00, B01, B02, B03};

u8 clock_phase;
u16 clock_time, clock_temp;

static softTimer_t triggerTimer = { .next = NULL, .prev = NULL };
u8 triggersBusy = 0;

u16 adc[4];
u8 front_timer;
u16 cv0, cv1;

u16 encoderDelta[4] = {0, 0, 0, 0};
u8 valueToShow = 0;
u8 prevPotValue = 16;

u8 isScaleEditing = 0, isScalePreview = 0;
u8 editButton1 = 0, editButton2 = 0;
u8 currentScaleRow = 0, currentScaleColumn = 0;

u8 arc2index = 0; // 0 - show rings 1&2, 1 - show rings 3&4
u8 isArc;
u8 gridParam = 0;
u8 scale;
u16 counter[4] = {0, 0, 0, 0};

u8 isDivisorArc, isPhaseArc, isChanceArc, isMixerArc;
u8 divisorArc, phaseArc, chanceArc, mixerArc;
u8 divisor[4], phase[4], reset[4], chance[4];
u8 mixerA, mixerB;

typedef const struct {
	u8 fresh;
	u8 isDivisorArc, isPhaseArc, isChanceArc, isMixerArc;
	u8 divisorArc, phaseArc, chanceArc, mixerArc;
	u8 divisor[4], phase[4], reset[4], chance[4];
	u8 mixerA, mixerB;
    u16 scales[16][16];
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
void initializeValues(void);

u16 indexToNote(u8 index);
u8 noteToIndex(u16 note);

void redraw(void);
void redrawArc(void);
void redrawGrid(void);
void updateOutputs(void);
void showValue(u8 value);
static void triggerTimer_callback(void* o);

////////////////////////////////////////////////////////////////////////////////
// application clock code

u16 indexToNote(u8 index)
{
	return CHROMATIC[index];
}

u8 noteToIndex(u16 note)
{
	u8 index = 0;
	for (u8 i = 0; i < 36; i++)
	{
		if (CHROMATIC[i] == note)
		{
			index = i;
			break;
		}
	}
	return index;
}

void redraw(void)
{
	isArc ? redrawArc() : redrawGrid();
	monomeFrameDirty = 0b1111;
}

void redrawGrid(void)
{
	u16 current, currentOn;

	if (isScaleEditing)
	{
		monomeLedBuffer[0] = monomeLedBuffer[16] = 15;
		monomeLedBuffer[32] = 0; monomeLedBuffer[48] = isScalePreview ? 8 : 0;
		u8 noteIndex;
		for (u8 y = 0; y < 4; y++)
		{
			for (u8 x = 0; x < 4; x++)
				monomeLedBuffer[64+(y<<4)+x] = currentScaleColumn == x ? 10 : 5;
			if (currentScaleRow == y) monomeLedBuffer[64+(y<<4)+currentScaleColumn] = 15;
			
			for (u8 col = 0; col < 12; col++)
			{
				monomeLedBuffer[68+(y<<4)+col] = 0;
			}
			noteIndex = noteToIndex(SCALES[scale][(y<<2)+currentScaleColumn]);
			monomeLedBuffer[68+(y<<4)+(noteIndex % 12)] = 5 + ((noteIndex / 12) * 5);
		}
		
		u8 cvA = 0;
		for (u8 seq = 0; seq < 4; seq++)
		{
			current = counter[seq] + (divisor[seq] << 6) - phase[seq];
			currentOn = (current / divisor[seq]) & 1;
			if (currentOn && (mixerA & (1 << seq))) cvA += 1 << seq;
		}
		monomeLedBuffer[64+((cvA&12)<<2)+(cvA&3)] = 15;
	}
	else
	{
		// currently selected grid parameter
		monomeLedBuffer[0] = gridParam == DIVISOR ? 15 : 6;
		monomeLedBuffer[16] = gridParam == PHASE ? 15 : 6;
		monomeLedBuffer[32] = gridParam == RESET ? 15 : 6;
		monomeLedBuffer[48] = gridParam == CHANCE ? 15 : 6;
		switch(gridParam)
		{
			case DIVISOR:
				for (u8 i = 0; i < 4; i++)
					for (u8 led = 0; led < 16; led++)
						monomeLedBuffer[64+(i<<4)+led] = led < divisor[i] ? 15 : 0;
				break;
			case PHASE:
				for (u8 i = 0; i < 4; i++)
					for (u8 led = 0; led < 16; led++)
						monomeLedBuffer[64+(i<<4)+led] = led < phase[i] ? 15 : 0;
				break;
			case RESET:
				for (u8 i = 0; i < 4; i++)
					for (u8 led = 0; led < 16; led++)
						monomeLedBuffer[64+(i<<4)+led] = led < reset[i] ? 15 : 0;
				break;
			case CHANCE:
				for (u8 i = 0; i < 4; i++)
					for (u8 led = 0; led < 16; led++)
						monomeLedBuffer[64+(i<<4)+led] = led < chance[i] ? 15 : 0;
				break;
		}
	}
	
	u8 seqOffset;
	for (u8 seq = 0; seq < 4; seq++)
	{
		seqOffset = seq << 4;
		for (u8 led = 1; led <= 13; led++)
		{
			current = counter[seq] + (divisor[seq] << 4) + led - phase[seq];
			currentOn = current / divisor[seq];
			monomeLedBuffer[14 - led + seqOffset] = (currentOn & 1) * 8;
		}
		
		current = counter[seq] + (divisor[seq] << 4) - phase[seq];
		currentOn = current / divisor[seq];
		monomeLedBuffer[14 + seqOffset] = (mixerA & (1 << seq)) ? ((currentOn & 1) ? 15 : 8) : 0;
		monomeLedBuffer[15 + seqOffset] = (mixerB & (1 << seq)) ? ((currentOn & 1) ? 15 : 8) : 0;
	}
	
	if (valueToShow == 5) // show scale
	{
		for (u8 led = 0; led < 16; led++)
			monomeLedBuffer[led+112] = led == scale ? 15 : 6;
	}
}

void redrawArc(void)
{
	u8 prev, next, level, currentLed;
	u16 current, currentOn;
	
	for (u8 enc = 0; enc < 4; enc++)
	{
		prev = 0;
		level = 2;
		
		for (u8 led = 0; led < 64; led++)
		{
			current = counter[enc] + (divisor[enc] << 6) - 32 + led - phase[enc];
			currentOn = current / divisor[enc];
			currentLed = ((counter[enc] + 64 - 32 + led) & 63) + (enc << 6);

			next = currentOn & 1;
			if (!prev && next)
			{
				level = 10 - (abs(led - 32) >> 2);
			}
			prev = next;
			
			monomeLedBuffer[currentLed] = next * level;
		}

		monomeLedBuffer[(enc << 6) + (counter[enc] & 63)] = 0xF;
	}

	if (monome_encs() == 2) // arc2
	{
		if (valueToShow == 1) // divisorArc 0-15
		{
			level = isDivisorArc ? 15 : 0;
			for (u8 led = 0; led < 64; led++)
				monomeLedBuffer[led] = !(led & 3) ? 5 : ((led < ((divisorArc + 1) << 2)) && (led >= (divisorArc << 2)) ? level : 0);
		}
		if (valueToShow == 2) // phaseArc 0-15
		{
			level = isPhaseArc ? 15 : 0;
			for (u8 led = 0; led < 64; led++)
				monomeLedBuffer[led] = !(led & 3) ? 5 : ((led < ((phaseArc + 1) << 2)) && (led >= (phaseArc << 2)) ? level : 0);
		}
		if (valueToShow == 3) // chanceArc 0-15
		{
			level = isChanceArc ? 15 : 0;
			for (u8 led = 0; led < 64; led++)
				monomeLedBuffer[led + 64] = !(led & 3) ? 5 : ((led < ((chanceArc + 1) << 2)) && (led >= (chanceArc << 2)) ? level : 0);
		}
		if (valueToShow == 4) // mixerArc 0-15
		{
			level = isMixerArc ? 15 : 0;
			for (u8 led = 0; led < 64; led++)
				monomeLedBuffer[led + 64] = !(led & 3) ? 5 : ((led < ((mixerArc + 1) << 2)) && (led >= (mixerArc << 2)) ? level : 0);
		}
		if (valueToShow == 5) // scale 0-15
		{
			for (u8 led = 0; led < 64; led++)
				monomeLedBuffer[led] = !(led & 3) ? 5 : ((led < ((scale + 1) << 2)) && (led >= (scale << 2)) ? 15 : 0);
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
		if (valueToShow == 1 || valueToShow == 6) // divisorArc 0-15
		{
			level = isDivisorArc ? 15 : 0;
			for (u8 led = 0; led < 64; led++)
				monomeLedBuffer[led] = !(led & 3) ? 5 : ((led < ((divisorArc + 1) << 2)) && (led >= (divisorArc << 2)) ? level : 0);
		}
		if (valueToShow == 2 || valueToShow == 6) // phaseArc 0-15
		{
			level = isPhaseArc ? 15 : 0;
			for (u8 led = 0; led < 64; led++)
				monomeLedBuffer[led + 64] = !(led & 3) ? 5 : ((led < ((phaseArc + 1) << 2)) && (led >= (phaseArc << 2)) ? level : 0);
		}
		if (valueToShow == 3 || valueToShow == 6) // chanceArc 0-15
		{
			level = isChanceArc ? 15 : 0;
			for (u8 led = 0; led < 64; led++)
				monomeLedBuffer[led + 128] = !(led & 3) ? 5 : ((led < ((chanceArc + 1) << 2)) && (led >= (chanceArc << 2)) ? level : 0);
		}
		if (valueToShow == 4 || valueToShow == 6) // mixerArc 0-15
		{
			level = isMixerArc ? 15 : 0;
			for (u8 led = 0; led < 64; led++)
				monomeLedBuffer[led + 192] = !(led & 3) ? 5 : ((led < ((mixerArc + 1) << 2)) && (led >= (mixerArc << 2)) ? level : 0);
		}
		if (valueToShow == 5) // scale 0-15
		{
			for (u8 led = 0; led < 64; led++)
				monomeLedBuffer[led] = !(led & 3) ? 5 : ((led < ((scale + 1) << 2)) && (led >= (scale << 2)) ? 15 : 0);
		}
	}
}

void updateOutputs()
{
	if (!triggersBusy) timer_remove(&triggerTimer);

	if (isScaleEditing && isScalePreview)
	{
		if (!triggersBusy) 
		{
			gpio_set_gpio_pin(TRIGGERS[0]);
			gpio_set_gpio_pin(TRIGGERS[1]);
			gpio_set_gpio_pin(TRIGGERS[2]);
			gpio_set_gpio_pin(TRIGGERS[3]);
		}
		cv0 = SCALES[scale][(currentScaleRow<<2)+currentScaleColumn];
		cv1 = 0;
	}
	else
	{
		u8 cvA = 0, cvB = 0;
		u16 prevOn, currentOn, offset;
			
		for (u8 seq = 0; seq < 4; seq++)
		{
			offset = counter[seq] + (divisor[seq] << 6) - phase[seq];
			currentOn = (offset / divisor[seq]) & 1;
			prevOn = ((offset - 1) / divisor[seq]) & 1;
			
			if (chance[seq] < ((rnd() % 20)+1))
			{
				if (currentOn && (mixerA & (1 << seq))) cvA += 1 << seq;
				if (currentOn && (mixerB & (1 << seq))) cvB += 1 << seq;
				if (!triggersBusy) prevOn != currentOn ? gpio_set_gpio_pin(TRIGGERS[seq]) : gpio_clr_gpio_pin(TRIGGERS[seq]);
			}
		}

		cv0 = SCALES[scale][cvA];
		cv1 = SCALES[scale][cvB];
	}

	if (!triggersBusy) timer_add(&triggerTimer, 10, &triggerTimer_callback, NULL);
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

void clock(u8 phase) {
	if(phase) {
		gpio_set_gpio_pin(B10);
		
		for (u8 seq = 0; seq < 4; seq++)
		{
			counter[seq]++;
			if (reset[seq] && (reset[seq] == counter[seq]))
				counter[seq] = 0;
			else if (counter[seq] >= ((u16)divisor[seq] << 6)) 
				counter[seq] = 0;
		}
		updateOutputs();
		redraw();
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
	redraw();
}

void showValue(u8 value)
{
	if (valueToShow) timer_remove(&showValueTimer);
	valueToShow = value;
	timer_add(&showValueTimer, 1000, &showValueTimer_callback, NULL);
	redraw();
}

static void triggerTimer_callback(void* o) {
	triggersBusy = 0;
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
	isArc = monome_device() == eDeviceArc;
	if (monome_device() == eDeviceGrid)
	{
		isDivisorArc = isPhaseArc = isChanceArc = isMixerArc = 0;
	}
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
		
		if (isArc)
		{
			if (monome_encs() == 2) // arc2
			{
				arc2index = !arc2index;
				redraw();
			}
			else
			{
				if (valueToShow == 6)
				{
					reset[0] = reset[1] = reset[2] = reset[3] = 0;
					counter[0] = counter[1] = counter[2] = counter[3] = 0;
				}
				else showValue(6);
			}
		}
		else
		{
			counter[0] = counter[1] = counter[2] = counter[3] = 0;
			redraw();
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
		showValue(5);
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
// application arc/grid code

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
					if (!isDivisorArc)
					{
						isDivisorArc = 1;
						divisorArc = 0;
					} else if (++divisorArc > 15) divisorArc = 0;
					for (u8 seq = 0; seq < 4; seq++) divisor[seq] = DIVISOR_PRESETS[divisorArc][seq];
					for (u8 seq = 0; seq < 4; seq++) counter[seq] = phase[seq] ? counter[0] % phase[seq] : counter[0] & 63;
					showValue(1);
				} 
				else 
				{ 
					if (!isPhaseArc)
					{
						isPhaseArc = 1;
						phaseArc = 0;
					}
					else if (++phaseArc > 15) phaseArc = 0;
					for (u8 seq = 0; seq < 4; seq++) phase[seq] = PHASE_PRESETS[phaseArc][seq];
					showValue(2);
				}
				break;
			case 1:
				if (delta < 0)
				{
					if (!isChanceArc)
					{
						isChanceArc = 1;
						chanceArc = 0;
					} else if (++chanceArc > 15) chanceArc = 0; 
					for (u8 seq = 0; seq < 4; seq++) chance[seq] = chanceArc;
					showValue(3);
				}
				else
				{
					if (!isMixerArc)
					{
						isMixerArc = 1;
						mixerArc = 0;
					} else if (++mixerArc > 15) mixerArc = 0;
					mixerA = MIXER_PRESETS[mixerArc][0];
					mixerB = MIXER_PRESETS[mixerArc][1];
					showValue(4);
				}
				break;
		}
	}
	else
	{
		switch(n)
		{
			case 0:
				if (!isDivisorArc)
				{
					isDivisorArc = 1;
					divisorArc = 0;
				}
				else if (delta > 0)
				{
					if (++divisorArc > 15) divisorArc = 0;
				}
				else
				{
					if (divisorArc > 0) divisorArc--; else divisorArc = 15;
				}
				
				for (u8 seq = 0; seq < 4; seq++) divisor[seq] = DIVISOR_PRESETS[divisorArc][seq];
				for (u8 seq = 0; seq < 4; seq++) counter[seq] = phase[seq] ? counter[0] % phase[seq] : counter[0] & 63;
				showValue(1);
				break;
			case 1:
				if (!isPhaseArc)
				{
					isPhaseArc = 1;
					phaseArc = 0;
				}
				else if (delta > 0)
				{
					if (++phaseArc > 15) phaseArc = 0;
				}
				else
				{
					if (phaseArc > 0) phaseArc--; else phaseArc = 15;
				}
				for (u8 seq = 0; seq < 4; seq++) phase[seq] = PHASE_PRESETS[phaseArc][seq];
				showValue(2);
				break;
			case 2:
				if (!isChanceArc)
				{
					isChanceArc = 1;
					chanceArc = 0;
				}
				else if (delta > 0)
				{
					if (++chanceArc > 15) chanceArc = 0;
				}
				else
				{
					if (chanceArc > 0) chanceArc--; else chanceArc = 15;
				}
				for (u8 seq = 0; seq < 4; seq++) chance[seq] = chanceArc;
				showValue(3);
				break;
			case 3:
				if (!isMixerArc)
				{
					isMixerArc = 1;
					mixerArc = 0;
				}
				else if (delta > 0)
				{
					if (++mixerArc > 15) mixerArc = 0;
				}
				else
				{
					if (mixerArc > 0) mixerArc--; else mixerArc = 15;
				}
				mixerA = MIXER_PRESETS[mixerArc][0];
				mixerB = MIXER_PRESETS[mixerArc][1];
				showValue(4);
				break;
		}
	}
}

static void handler_MonomeGridKey(s32 data) { 
	u8 x, y, z;
	monome_grid_key_parse_event_data(data, &x, &y, &z);
	// z == 0 key up, z == 1 key down
	
	if (!z) 
    {
        if (x == 0 && y == 0) editButton1 = 0;
        if (x == 0 && y == 1) editButton2 = 0;
        return;
    }
	
	if (x == 0 && y < 4) // grid param select
	{
        if (y == 0) editButton1 = 1;
        if (y == 1) editButton2 = 1;
        
		if (isScaleEditing && (y == 3)) // scale preview
		{
			isScalePreview = !isScalePreview;
			redraw();
			return;
		}
		
        if (editButton1 && editButton2)
        {
            isScaleEditing = 1;
            redraw();
            return;
        }
        
		isScaleEditing = 0;
		
		if (y == 0) gridParam = DIVISOR;
		else if (y == 1) gridParam = PHASE;
		else if (y == 2) gridParam = RESET;
		else if (y == 3) gridParam = CHANCE;
		
		redraw();
		return;
	}
	
	if (x == 14 && y < 4) // CV A mixing
	{
		if (mixerA & (1 << y))
			mixerA &= ~(1 << y);
		else
			mixerA |= (1 << y);
		redraw();
		return;
	}

	if (x == 15 && y < 4) // CV B mixing
	{
		if (mixerB & (1 << y))
			mixerB &= ~(1 << y);
		else
			mixerB |= (1 << y);
		redraw();
		return;
	}
	
	if (y < 4) return;
	
	if (isScaleEditing)
	{
		currentScaleRow = y - 4;
		if (x < 4)
		{
			currentScaleColumn = x;
			redraw();
			return;
		}
		
		u8 noteIndex = noteToIndex(SCALES[scale][(currentScaleRow<<2)+currentScaleColumn]);
		if (noteIndex % 12 == x - 4)
		{
			noteIndex = (noteIndex + 12) % 36;
		}
		else
		{
			noteIndex = x - 4;
		}
		SCALES[scale][(currentScaleRow<<2)+currentScaleColumn] = indexToNote(noteIndex);
		redraw();
		return;
	}
	else // param editing
	{
		switch (gridParam)
		{
			case DIVISOR:
				divisor[y - 4] = x + 1;
				break;
			case PHASE:
				if (phase[y - 4] == x + 1)
					phase[y - 4] = 0;
				else
					phase[y - 4] = x + 1;
				break;
			case RESET:
				if (reset[y - 4] == x + 1)
				{
					reset[y - 4] = 0;
				}
				else
				{
					reset[y - 4] = x + 1;
					counter[y - 4] = counter[y - 4] % reset[y - 4];
				}
				break;
			case CHANCE:
				if (chance[y - 4] == x + 1)
					chance[y - 4] = 0;
				else
					chance[y - 4] = x + 1;
				break;
		}
		redraw();
		return;
	}
}
////////////////////////////////////////////////////////////////////////////////

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
	app_event_handlers[ kEventMonomeGridKey ]	= &handler_MonomeGridKey ;
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
	flashc_memset8((void*)&(flashy.isDivisorArc), isDivisorArc, 1, true);
	flashc_memset8((void*)&(flashy.isPhaseArc), isPhaseArc, 1, true);
	flashc_memset8((void*)&(flashy.isChanceArc), isChanceArc, 1, true);
	flashc_memset8((void*)&(flashy.isMixerArc), isMixerArc, 1, true);
	flashc_memset8((void*)&(flashy.divisorArc), divisorArc, 1, true);
	flashc_memset8((void*)&(flashy.phaseArc), phaseArc, 1, true);
	flashc_memset8((void*)&(flashy.chanceArc), chanceArc, 1, true);
	flashc_memset8((void*)&(flashy.mixerArc), mixerArc, 1, true);
	flashc_memset8((void*)&(flashy.mixerA), mixerA, 1, true);
	flashc_memset8((void*)&(flashy.mixerB), mixerB, 1, true);
	flashc_memcpy((void *)&flashy.divisor, &divisor, sizeof(divisor), true);
	flashc_memcpy((void *)&flashy.phase, &phase, sizeof(phase), true);
	flashc_memcpy((void *)&flashy.reset, &reset, sizeof(reset), true);
	flashc_memcpy((void *)&flashy.chance, &chance, sizeof(chance), true);
    
    for (u8 sc = 0; sc < 16; sc++)
		flashc_memcpy((void *)&flashy.scales[sc], &SCALES[sc], sizeof(SCALES[sc]), true);
	
	triggersBusy = 1;
	timer_remove(&triggerTimer);
	for (u8 enc = 0; enc < 4; enc++)
		gpio_set_gpio_pin(TRIGGERS[enc]);
	timer_add(&triggerTimer, 400, &triggerTimer_callback, NULL);
}

void flash_read(void) {
	isDivisorArc = flashy.isDivisorArc;
	isPhaseArc = flashy.isPhaseArc;
	isChanceArc = flashy.isChanceArc;
	isMixerArc = flashy.isMixerArc;
	divisorArc = flashy.divisorArc;
	phaseArc = flashy.phaseArc;
	chanceArc = flashy.chanceArc;
	mixerArc = flashy.mixerArc;
	mixerA = flashy.mixerA;
	mixerB = flashy.mixerB;
	for (u8 i = 0; i < 4; i++)
	{
		divisor[i] = flashy.divisor[i];
		phase[i] = flashy.phase[i];
		reset[i] = flashy.reset[i];
		chance[i] = flashy.chance[i];
	}
    for (u8 sc = 0; sc < 16; sc++)
		for (u8 i = 0; i < 16; i++)
			SCALES[sc][i] = flashy.scales[sc][i];
}

void initializeValues(void)
{
	isDivisorArc = isPhaseArc = isChanceArc = isMixerArc = divisorArc = phaseArc = chanceArc = mixerArc = 0;
	mixerA = MIXER_PRESETS[0][0];
	mixerB = MIXER_PRESETS[0][1];
	for (u8 i = 0; i < 4; i++)
	{
		divisor[i] = DIVISOR_PRESETS[0][i];
		phase[i] = PHASE_PRESETS[0][i];
		reset[i] = 0;
		chance[i] = 0;
	}
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
		initializeValues();
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

	updateOutputs();
	redraw();
	
	while (true) {
		check_events();
	}
}
