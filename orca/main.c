#include <stdio.h>
#include <ctype.h>  //toupper
#include <string.h> //memcpy

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

#include "uhi_msc.h"
#include "fat.h"
#include "file.h"
#include "fs_com.h"
#include "navigation.h"
#include "usb_protocol_msc.h"
#include "uhi_msc_mem.h"

// skeleton
#include "types.h"
#include "events.h"
#include "i2c.h"
#include "init.h"
#include "interrupts.h"
#include "monome.h"
#include "timers.h"
#include "adc.h"
#include "util.h"
#include "ftdi.h"

// this
#include "conf_board.h"
#include "ii.h"


	
#define FIRSTRUN_KEY 0x51
#define ENCODER_DELTA_SENSITIVITY 40
#define NOTEMAX 60

#define DIVISOR 0
#define PHASE 1
#define RESET 2
#define CHANCE 3

#define SCALE 4
#define SETTINGS 5
#define RANDOMIZE 6
#define PRESETS 7

#define ROTATESCALE 8
#define ROTATEWEIGHTS 9
#define MUTATESEQ 10
#define GLOBALRESET 11

#define COUNTERS 12
#define DEBUG 13


u8 SCALE_PRESETS[16][16] = {
    {0, 2, 4, 5, 7, 9, 11, 12, 14, 16, 17, 19, 21, 23, 24, 26}, // ionian [2, 2, 1, 2, 2, 2, 1]
    {0, 2, 3, 5, 7, 9, 10, 12, 14, 15, 17, 19, 21, 22, 24, 26}, // dorian [2, 1, 2, 2, 2, 1, 2]
    {0, 1, 3, 5, 7, 8, 10, 12, 13, 15, 17, 19, 20, 22, 24, 25}, // phrygian [1, 2, 2, 2, 1, 2, 2]
    {0, 2, 4, 6, 7, 9, 11, 12, 14, 16, 18, 19, 21, 23, 24, 26}, // lydian [2, 2, 2, 1, 2, 2, 1]
    {0, 2, 4, 5, 7, 9, 10, 12, 14, 16, 17, 19, 21, 22, 24, 26}, // mixolydian [2, 2, 1, 2, 2, 1, 2]
    {0, 2, 4, 5, 7, 9, 10, 12, 14, 16, 17, 19, 21, 22, 24, 26}, // aeolian [2, 1, 2, 2, 1, 2, 2]
    {0, 1, 3, 5, 6, 8, 10, 12, 13, 15, 17, 18, 20, 22, 24, 25}, // locrian [1, 2, 2, 1, 2, 2, 2]
    {0, 2, 3, 6, 7, 9, 10, 12, 14, 15, 18, 19, 21, 22, 24, 26}, // Ukrainian Dorian

    {0, 1, 4, 5, 6, 8, 11, 12, 13, 16, 17, 18, 20, 23, 24, 25}, // Persian
    {9, 11, 12, 15, 16, 17, 19, 21, 23, 24, 27, 28, 29, 31, 33, 35}, // Hungarian
    {7, 8, 11, 13, 15, 17, 18, 19, 20, 23, 25, 27, 29, 30, 31, 32}, // enigmatic
    {0, 1, 4, 5, 7, 8, 10, 12, 13, 16, 17, 19, 20, 22, 24, 25}, // Phrygian dominant
    {0, 1, 4, 6, 8, 10, 12, 13, 16, 18, 20, 22, 24, 25, 28, 30}, // Tritone 
    {0, 2, 4, 6, 7, 9, 10, 12, 14, 16, 18, 19, 21, 22, 24, 26}, // Acoustic 
    {0, 1, 3, 4, 6, 8, 10, 12, 13, 15, 16, 18, 20, 22, 24, 25},  // Altered 
    {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15}	// chromatic
};

const u16 CHROMATIC[NOTEMAX] = {
	   0,   34,   68,  102,  136,  170,  204,  238,  272,  306,  340,  375,
	 409,  443,  477,  511,  545,  579,  613,  647,  681,  715,  750,  784,
	 818,  852,  886,  920,  954,  988, 1022, 1056, 1090, 1125, 1159, 1193,
	1227, 1261, 1295, 1329, 1363, 1397, 1431, 1465, 1500, 1534, 1568, 1602,
	1636, 1670, 1704, 1738, 1772, 1806, 1841, 1875, 1909, 1943, 1977, 2011
};

const u8 DIVISOR_PRESETS[16][4] =
{
    {1, 2, 3, 4},
	{1, 2, 4, 8},
	{2, 4, 8, 16},
	{2, 3, 4, 5},

    {2, 3, 5, 7},
    {2, 3, 5, 8},
    {3, 5, 7, 9},
    {3, 6, 9, 12},

    {12, 9, 6, 3},
    {9, 7, 5, 3},
    {8, 5, 3, 2},
    {7, 5, 3, 2},

    {5, 4, 3, 2},
    {16, 8, 4, 2},
    {8, 4, 2, 1},
    {4, 3, 2, 1}
};

const u8 PHASE_PRESETS[16][4] =
{
	{0, 0, 0, 0},
	{0, 1, 2, 3},
	{1, 2, 3, 4},
	{2, 3, 4, 5},
	
	{0, 2, 4, 8},
	{0, 3, 6, 9},
	{0, 1, 3, 5},
	{0, 2, 3, 5},
	
	{5, 3, 2, 0},
	{5, 3, 1, 0},
	{9, 6, 3, 0},
	{8, 4, 2, 0},
	
	{5, 4, 3, 2},
	{4, 3, 2, 1},
	{3, 2, 1, 0},
	{1, 1, 1, 1}
};

const u8 MIXER_PRESETS[16][2] =
{
	{0b1110, 0b0001},
	{0b1101, 0b0010},
	{0b1011, 0b0100},
	{0b0111, 0b1000},
	
	{0b1010, 0b0101},
	{0b0101, 0b1010},
	{0b1001, 0b0110},
	{0b0110, 0b1001},

	{0b1110, 0b0111},
	{0b0111, 0b1110},
	{0b1011, 0b1101},
	{0b1101, 0b1011},

	{0b1111, 0b0011},
	{0b1111, 0b0110},
	{0b1111, 0b1100},
	{0b1111, 0b1001}
};

const u8 TRIGGERS[4] = {B00, B01, B02, B03};

u8 clock_phase;
u16 clock_time, clock_temp;

static softTimer_t triggerTimer[4] = {{.next = NULL, .prev = NULL}, {.next = NULL, .prev = NULL}, {.next = NULL, .prev = NULL}, {.next = NULL, .prev = NULL}};
static softTimer_t confirmationTimer = { .next = NULL, .prev = NULL };
static softTimer_t cvPreviewTimer = { .next = NULL, .prev = NULL };
static softTimer_t triggerSettingsTimerG = { .next = NULL, .prev = NULL };
static softTimer_t triggerSettingsTimer[4] = {{.next = NULL, .prev = NULL}, {.next = NULL, .prev = NULL}, {.next = NULL, .prev = NULL}, {.next = NULL, .prev = NULL}};
static softTimer_t flashSavedTimer = { .next = NULL, .prev = NULL };
static softTimer_t doublePressTimer = { .next = NULL, .prev = NULL };

u16 adc[4];
u8 front_timer;
u16 cv0, cv1;

s16 encoderDelta[4] = {0, 0, 0, 0};
u8 valueToShow = 0;
u8 frontClicked = 0;
u8 prevPotValue = 16;

u8 isScalePreview = 0;
u8 scalePreviewEnabled = 0;
u8 scaleBlink = 0;
u8 prevSelectedScaleColumn = 4;
u8 currentScaleRow = 0, currentScaleColumn = 0;
int debug[4] = {0, 0, 0, 0};
char str[256], svalue[256];
u8 bankToLoad = 0, presetToLoad = 0;

u8 arc2index = 0; // 0 - show tracks 1&2, 1 - show tracks 3&4, 2 - show scales
u8 arc4index = 0; // 0 - show tracks, 1 - show values, 2 - show scales
u8 isArc, isVb, isArc2;
u8 gridParam = DIVISOR;
u8 isDivisorArc = 0, isPhaseArc = 0, isMixerArc = 0;
u64 globalCounter = 0, globalLength;
u16 counter[4] = {0, 0, 0, 0};
u8 triggerOn[4] = {0, 0, 0, 0};
u16 prevOn[4] = {0, 0, 0, 0};
u8 flashConfirmation = 0;
u8 showTriggersG = 0, showRandomized = 0, bankToShow = 8, presetToShow = 8, presetPressed = 8, bankPressed = 8, notePressedIndex = 12, noteSquarePressedRow = 4, noteSquarePressedCol = 4, lastSelectedScale = 0;
u8 showTriggers[4] = {0, 0, 0, 0};
u8 scalePressed = 0, scalePresetPressed = 16, userScalePressed = 16, presetModePressed = 0;
u8 prevXPressed = 16, prevXReleased = 16, prevYPressed = 8, prevYReleased = 8;
u8 randomizeX = 16, randomizeY = 4;
u8 iiTrack = 0;

struct preset {
	u8 scaleA, scaleB;
	u8 scales[16][16];
	u8 divisorArc, phaseArc, mixerArc;
	u8 divisor[4], phase[4], reset[4], chance[4], weight[4];
	u8 gateType[4], gateMuted[4], gateLogic[4], gateNot[4], gateTracks[4];
	u8 mixerA, mixerB, alwaysOnA, alwaysOnB;
	s8 rotateScale[16], rotateWeights[16]; 
	u8 mutateSeq[8];
	u8 globalReset;
};

struct bank {
	u8 cp;
	struct preset presets[8];
} banks[8];

u8 cb;

u8 scaleA, scaleB;
u8 scales[16][16];
u8 divisorArc, phaseArc, mixerArc;
u8 divisor[4], phase[4], reset[4], chance[4], weight[4];
u8 gateType[4], gateMuted[4], gateLogic[4], gateNot[4], gateTracks[4];
u8 mixerA, mixerB, alwaysOnA, alwaysOnB;
s8 rotateScale[16], rotateWeights[16]; 
u8 mutateSeq[8];
u8 globalReset;

u8 userScalePresets[16][16];
u8 divisor_presets[16][4];
u8 phase_presets[16][4];
u8 mixer_presets[16][2];

typedef const struct {
	u8 fresh;
	u8 userScalePresets[16][16];
	u8 divisor_presets[16][4];
	u8 phase_presets[16][4];
	u8 mixer_presets[16][2];
    u8 currentBank;
	struct bank banks[8];
} nvram_data_t;
static nvram_data_t flashy;

// NVRAM data structure located in the flash array.
__attribute__((__section__(".flash_nvram")))
static nvram_data_t flashy;

typedef void(*trigger_c)(void* o); 

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
static void orca_process_ii(uint8_t i, int d);

static void usb_stick_save(void);
static void usb_stick_load(void);
static void usb_stick(u8 includeLoading);
void process_line(void);
void usb_write_str(char* str);
void usb_write_param(char* name, u16 value, int base);
void usb_write_u8_array(char* name, u8* array, u8 length, int base, u8 padding);
void usb_write_s8_array(char* name, s8* array, u8 length);
u8 load_u8(char* s, u8 min, u8 max);
u8 load_u8_bin(char* s, u8 min, u8 max);
void load_u8_array(char* s, u8* array, u8 len, u8 min, u8 max);
void load_u8_array_bin(char* s, u8* array, u8 len, u8 min, u8 max);
void load_s8_array(char* s, s8* array, u8 len, s8 min, s8 max);
u8 flash_is_fresh(void);
void flash_unfresh(void);
void flash_write(void);
void flash_read(void);
void initializeValues(void);

void updateglobalLength(void);
void adjustCounter(u8 index);
void adjustAllCounters(void);
u8 getMutateSeq(u8 index);
void setMutateSeq(u8 index);
void clrMutateSeq(u8 index);
u8 random8(void);
void generateChaos(void);
void generateRandom(u8 max, u8 paramCount);
void mutate(void);
void rotateScales(s8 amount);
void rotateWeights_(s8 amount);
void loadPreset(u8 b, u8 p);
void loadBank(u8 b, u8 updatecp);
void copyBank(u8 source, u8 dest);
void copyPreset(u8 source, u8 dest);
void updatePresetCache(void);
void determineArcSets(void);
u8 isTrackDead(u8 divisor, u8 phase, u8 reset, u8 globalReset);

void redraw(void);
void redrawArc(void);
void redrawGrid(void);
void showValue(u8 value);

void updateCVOutputs(u8 fromClock);
void updateTriggerOutputs(u8 fromClock);

static void triggerTimer0_callback(void* o);
static void triggerTimer1_callback(void* o);
static void triggerTimer2_callback(void* o);
static void triggerTimer3_callback(void* o);
trigger_c triggerTimer_callbacks[4];

static void confirmationTimer_callback(void* o);
static void cvPreviewTimer_callback(void* o);

static void triggerSettingsTimerG_callback(void* o);
static void triggerSettingsTimer0_callback(void* o);
static void triggerSettingsTimer1_callback(void* o);
static void triggerSettingsTimer2_callback(void* o);
static void triggerSettingsTimer3_callback(void* o);
trigger_c triggerSettingsTimer_callbacks[4];

static void flashSavedTimer_callback(void* o);
static void doublePressTimer_callback(void* o);

////////////////////////////////////////////////////////////////////////////////
// application clock code

u8 getMutateSeq(u8 index)
{
    return mutateSeq[index >> 3] & (1 << (index & 7));
}

void setMutateSeq(u8 index)
{
    mutateSeq[index >> 3] |= 1 << (index & 7);
	banks[cb].presets[banks[cb].cp].mutateSeq[index >> 3] = mutateSeq[index >> 3];
}

void clrMutateSeq(u8 index)
{
    mutateSeq[index >> 3] &= ~(1 << (index & 7));
	banks[cb].presets[banks[cb].cp].mutateSeq[index >> 3] = mutateSeq[index >> 3];
}

void redraw(void)
{
	isArc ? redrawArc() : redrawGrid();
	monomeFrameDirty = 0b1111;
}

void redrawGrid(void)
{
	u16 current, currentOn;
	u8 seqOffset;

	monomeLedBuffer[0] = monomeLedBuffer[16] = monomeLedBuffer[32] = monomeLedBuffer[48] = 5;
	monomeLedBuffer[1] = monomeLedBuffer[17] = monomeLedBuffer[33] = monomeLedBuffer[49] = 3;
	
	seqOffset = globalCounter & 15;
	monomeLedBuffer[2] = rotateScale[seqOffset] != 0 && showTriggersG ? 10 : 5;
	monomeLedBuffer[18] = rotateWeights[seqOffset] != 0 && showTriggersG ? 10 : 5;
	monomeLedBuffer[34] = getMutateSeq(globalCounter & 63) != 0 && showTriggersG ? 10 : 5;
	monomeLedBuffer[50] = globalReset && !globalCounter && showTriggersG ? 10 : 5;

    if (gridParam == COUNTERS)
    {
		for (u8 i = 0; i < 4; i++)
			for (u8 led = 0; led < 16; led++)
				monomeLedBuffer[64+(i<<4)+led] = led < (counter[i] & 15) ? 15 : 0;
    }
    else if (gridParam == DEBUG)
    {
		for (u8 i = 0; i < 4; i++)
		{
			u8 sign = debug[i] < 0 ? 7 : 15;
			u16 value = debug[i] < 0 ? (u16)(-debug[i]) : (u16)debug[i];
			/*
			for (u8 led = 0; led < 16; led++)
				monomeLedBuffer[64+(i<<4)+led] = led < value ? sign : 0;
			*/
			for (u8 led = 0; led < 16; led++)
				monomeLedBuffer[64+(i<<4)+led] = value & (1 << led) ? sign : 0;
		}
    }
    else if (gridParam == DIVISOR)
	{
		monomeLedBuffer[0] = 15;
		for (u8 i = 0; i < 4; i++)
			for (u8 led = 0; led < 16; led++)
				monomeLedBuffer[64+(i<<4)+led] = led < divisor[i] ? 15 : 0;
	}
	else if (gridParam == PHASE)
	{
		monomeLedBuffer[16] = 15;
		for (u8 i = 0; i < 4; i++)
			for (u8 led = 0; led < 16; led++)
				monomeLedBuffer[64+(i<<4)+led] = led < phase[i] ? 15 : 0;
	}
	else if (gridParam == RESET)
	{
		monomeLedBuffer[32] = 15;
		for (u8 i = 0; i < 4; i++)
			for (u8 led = 0; led < 16; led++)
				monomeLedBuffer[64+(i<<4)+led] = led < reset[i] ? 15 : 0;
	}
	else if (gridParam == CHANCE)
	{
		monomeLedBuffer[48] = 15;
		for (u8 i = 0; i < 4; i++)
			for (u8 led = 0; led < 16; led++)
				monomeLedBuffer[64+(i<<4)+led] = led < chance[i] ? 15 : 0;
	}
	else if (gridParam == SCALE)
	{
		monomeLedBuffer[1] = isScalePreview ? (scaleBlink ? 15 : 10) : 15;

		if (scalePressed)
        {
			for (u8 i = 0; i < 16; i++)
			{
				monomeLedBuffer[64 + i] = isVb ? i : ((i & 1) * 15);
				monomeLedBuffer[80 + i] = isVb ? 15 - i : (((i+1) & 1) * 15);
				monomeLedBuffer[96 + i] = i == scaleA ? (lastSelectedScale ? 8 : 15) : 0;
				monomeLedBuffer[112 + i] = i == scaleB ? (lastSelectedScale ? 15 : 8) : 0;
			}
        }
        else
        {
            u8 noteIndex;
            u8 cv = 0;
            
            for (u8 x = 0; x < 4; x++)
            {
                // note selection 4x4 on the left
                for (u8 y = 0; y < 4; y++)
                    monomeLedBuffer[64 + (y << 4) + x] = currentScaleRow == y ? 8 : 4;
                
                // clear note space
                for (u8 col = 0; col < 12; col++)
                {
                    monomeLedBuffer[68 + (x << 4) + col] = 0;
                }
                
                // display current note for the currently selected row
                noteIndex = scales[lastSelectedScale ? scaleB : scaleA][(currentScaleRow << 2) + x];
                monomeLedBuffer[68 + (x << 4) + (noteIndex % 12)] = 6 + ((noteIndex / 12) * 2);
            }
            
            // currently selected note in the 4x4 block
            monomeLedBuffer[64 + (currentScaleRow << 4) + currentScaleColumn] = 13;

            // highlight the current note
            if (!isScalePreview)
            {
                for (u8 seq = 0; seq < 4; seq++)
                {
                    current = counter[seq] + (divisor[seq] << 6) - phase[seq];
                    currentOn = (current / divisor[seq]) & 1;
					if (lastSelectedScale)
						{ if ((alwaysOnB & (1 << seq)) | (currentOn && (mixerB & (1 << seq)))) cv += weight[seq]; }
					else
						{ if ((alwaysOnA & (1 << seq)) | (currentOn && (mixerA & (1 << seq)))) cv += weight[seq]; }
                }
                cv &= 0xf;
                
                // highlight it in the 4x4 block
                monomeLedBuffer[64+((cv>>2)<<4)+(cv&3)] = 15;

                // highlight it in the note space
                if ((cv>>2) == currentScaleRow)
                {
                    noteIndex = scales[lastSelectedScale ? scaleB : scaleA][cv] % 12;
                    monomeLedBuffer[68 + ((cv & 3) << 4) + noteIndex]++;
                }
            }
        }    
	}
	else if (gridParam == SETTINGS)
	{
		monomeLedBuffer[17] = 15;
		
		u8 brighter, add[4];
		for (u8 i = 0; i < 4; i++)
		{
			current = counter[i] + (divisor[i] << 4) - phase[i];
			currentOn = current / divisor[i];
			add[i] = (currentOn & 1) ? 3 : 0;
		}
		
		// triggers/gates
		for (u8 i = 0; i < 4; i++)
		{
			seqOffset = (i << 4) + 64;
			brighter = triggerOn[i] || showTriggers[i] ? 3 : 0;
			
			monomeLedBuffer[seqOffset] = (gateMuted[i] ? 0 : 12) + brighter;
			monomeLedBuffer[seqOffset + 1] = gateType[i] * 6 + brighter;
			monomeLedBuffer[seqOffset + 2] = gateLogic[i] ? 12 : 0;
			monomeLedBuffer[seqOffset + 3] = gateNot[i] ? 12 : 0;
			
			monomeLedBuffer[seqOffset + 4] = (gateTracks[i] & 0b0001 ? 12 : 0) + add[0];
			monomeLedBuffer[seqOffset + 5] = (gateTracks[i] & 0b0010 ? 12 : 0) + add[1];
			monomeLedBuffer[seqOffset + 6] = (gateTracks[i] & 0b0100 ? 12 : 0) + add[2];
			monomeLedBuffer[seqOffset + 7] = (gateTracks[i] & 0b1000 ? 12 : 0) + add[3];
		}
		
		// track weights
		for (u8 i = 0; i < 4; i++)
			for (u8 led = 0; led < 8; led++)
				monomeLedBuffer[(i << 4) + 72 + led] = (led < weight[i] ? 12 : 0) + add[i];
	}
	else if (gridParam == RANDOMIZE)
	{
		monomeLedBuffer[33] = 15;
		
		if (showRandomized == 2) // chaos
		{
			for (u8 led = 0; led < 64; led++)
				monomeLedBuffer[64+led] = random8() & 15;
		}
		else
		{
			u8 brightness;
			for (u8 i = 0; i < 4; i++)
			{
				for (u8 led = 0; led < 16; led++)
				{
					brightness = isVb ? ((i << 1) + ((led * 10) >> 4)) : ((i + led) & 1) * 15;
					monomeLedBuffer[64+(i<<4)+led] = showRandomized && (i == randomizeY) ? 0 : brightness;
				}
				if (showRandomized) monomeLedBuffer[64+(i<<4)+randomizeX] = 0;
			}
			if (showRandomized) monomeLedBuffer[64+(randomizeY<<4)+randomizeX] = 15;
		}
	}
	else if (gridParam == PRESETS)
	{
		monomeLedBuffer[49] = 15;
        for (u8 i = 0; i < 8; i++)
        {
            monomeLedBuffer[64+(i<<1)] = 0;
            monomeLedBuffer[65+(i<<1)] = i == cb ? 13 : (i & 1 ? 5 : 0);
            monomeLedBuffer[80+(i<<1)] = i == cb ? 13 : (i & 1 ? 5 : 0);
            monomeLedBuffer[81+(i<<1)] = isVb ? (i == cb ? 13 : (i & 1 ? 5 : 0)) : 15;
        }
        for (u8 i = 0; i < 8; i++)
        {
            monomeLedBuffer[96+(i<<1)] = i == banks[cb].cp ? 13 : (i & 1 ? 0 : 5);
            monomeLedBuffer[97+(i<<1)] = i == banks[cb].cp ? 13 : (i & 1 ? 0 : 5);
            monomeLedBuffer[112+(i<<1)] = i == banks[cb].cp ? 13 : (i & 1 ? 0 : 5);
            monomeLedBuffer[113+(i<<1)] = isVb ? (i == banks[cb].cp ? 13 : (i & 1 ? 0 : 5)) : 15;
        }
		if (presetToShow != 8)
		{
			u8 i = ((presetToShow >> 3) << 5) + ((presetToShow & 7) << 1);
			monomeLedBuffer[96+i] = 15;
			monomeLedBuffer[97+i] = 15;
			monomeLedBuffer[112+i] = 15;
			monomeLedBuffer[113+i] = 15;
		}    
		if (bankToShow != 8)
		{
			u8 i = ((bankToShow >> 3) << 5) + ((bankToShow & 7) << 1);
			monomeLedBuffer[64+i] = 0;
			monomeLedBuffer[65+i] = 15;
			monomeLedBuffer[80+i] = 15;
			monomeLedBuffer[81+i] = 15;
		}    
	}
	else if (gridParam == ROTATESCALE)
	{
		monomeLedBuffer[2] = 15;
		u8 step = globalCounter & 15;
		u8 add;
		for (u8 i = 0; i < 16; i++)
		{
			add = step == i ? 3 : 0;
			monomeLedBuffer[64 + i] = (rotateScale[i] > 1 ? 12 : 0) + add;
			monomeLedBuffer[80 + i] = (rotateScale[i] > 0 ? 12 : 0) + add;
			monomeLedBuffer[96 + i] = (rotateScale[i] < 0 ? 12 : 0) + add;
			monomeLedBuffer[112 + i] = (rotateScale[i] < -1 ? 12 : 0) + add;
		}
	}
	else if (gridParam == ROTATEWEIGHTS)
	{
		monomeLedBuffer[18] = 15;
		u8 step = globalCounter & 15;
		u8 add;
		for (u8 i = 0; i < 16; i++)
		{
			add = step == i ? 3 : 0;
			monomeLedBuffer[64 + i] = (rotateWeights[i] > 1 ? 12 : 0) + add;
			monomeLedBuffer[80 + i] = (rotateWeights[i] > 0 ? 12 : 0) + add;
			monomeLedBuffer[96 + i] = (rotateWeights[i] < 0 ? 12 : 0) + add;
			monomeLedBuffer[112 + i] = (rotateWeights[i] < -1 ? 12 : 0) + add;
		}
	}
	else if (gridParam == MUTATESEQ)
	{
		monomeLedBuffer[34] = 15;
		u8 step = globalCounter & 63;
		u8 add;
		for (u8 i = 0; i < 64; i++)
		{
			add = step == i ? 3 : 0;
			monomeLedBuffer[64 + i] = (getMutateSeq(i) ? 12 : 0) + add;
		}
	}
	else if (gridParam == GLOBALRESET)
	{
		monomeLedBuffer[50] = 15;
		u8 step = globalCounter & 63;
		u8 add;
		for (u8 i = 0; i < 64; i++)
		{
			add = step == i ? 3 : 0;
			monomeLedBuffer[64 + i] = (i < globalReset ? 12 : 0) + add;
		}
	}
	
	// tracks
	u16 _counter;
	u64 _globalCounter;
	for (u8 seq = 0; seq < 4; seq++)
	{
		_globalCounter = globalCounter;
		_counter = counter[seq];
		seqOffset = seq << 4;
		for (u8 led = 0; led < 11; led++)
		{
			current = _counter + (divisor[seq] << 4) - phase[seq];
			currentOn = (current / divisor[seq]) & 1;
			monomeLedBuffer[13 - led + seqOffset] = (!isVb && led == 10) ? 0 : (currentOn ? (isVb ? 15 - led : 15) : 0);
			
            _globalCounter++;
            if (_globalCounter >= globalLength || (globalReset && _globalCounter >= globalReset))
			{
				_globalCounter = _counter = 0;
			}
			else
			{
				_counter++;
				if (reset[seq])
				{
					if (_counter >= reset[seq]) _counter = 0;
				}
				else
				{
					if (_counter >= ((u16)divisor[seq] << 6)) _counter = 0;
				}
			}
		}
		
		current = counter[seq] + (divisor[seq] << 4) - phase[seq];
		currentOn = (current / divisor[seq]) & 1;
		
		monomeLedBuffer[14 + seqOffset] = alwaysOnA & (1 << seq) ? 15 : (mixerA & (1 << seq) ? (currentOn ? 12 : (isVb ? 6 : 15)) : 0);
		monomeLedBuffer[15 + seqOffset] = alwaysOnB & (1 << seq) ? 15 : (mixerB & (1 << seq) ? (currentOn ? 12 : (isVb ? 6 : 15)) : 0);
		if (gridParam == SCALE)
		{
			if (lastSelectedScale)
				monomeLedBuffer[14 + seqOffset] = 0;
			else 
				monomeLedBuffer[15 + seqOffset] = 0;
		}
	}
	
	if (valueToShow == 5) // show scale
	{
		for (u8 led = 0; led < 16; led++)
		{
			monomeLedBuffer[led+96] = led == scaleA ? 15 : 0;
			monomeLedBuffer[led+112] = led == scaleB ? 15 : 0;
		}
			
	}
	
	if (flashConfirmation == 1)
	{
		for (u8 i = 0; i < 8; i++)
		{
			monomeLedBuffer[i << 4] = 15;
			monomeLedBuffer[(i << 4) + 1] = 15;
			monomeLedBuffer[(i << 4) + 14] = 15;
			monomeLedBuffer[(i << 4) + 15] = 15;
		}
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

	if (isArc2) // arc2
	{
		if (arc2index == 1)
		{
			for (u8 led = 0; led < 64; led++)
			{
				monomeLedBuffer[led] = monomeLedBuffer[led + 128];
				monomeLedBuffer[led + 64] = monomeLedBuffer[led + 192];
			}
		}
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
		if (valueToShow == 3) // mixerArc 0-15
		{
			level = isMixerArc ? 15 : 0;
			for (u8 led = 0; led < 64; led++)
				monomeLedBuffer[led + 64] = !(led & 3) ? 5 : ((led < ((mixerArc + 1) << 2)) && (led >= (mixerArc << 2)) ? level : 0);
		}
		if (valueToShow == 4) // preset 0-7
		{
			u8 pres = (cb << 3) + banks[cb].cp;
			for (u8 led = 0; led < 64; led++)
				monomeLedBuffer[led + 64] = led == pres ? 15 : (led & 7 ? 1 : 6);
		}
		if (valueToShow == 5 || arc2index == 2) // scale 0-15
		{
			for (u8 led = 0; led < 64; led++)
			{
				monomeLedBuffer[led] = !(led & 3) ? 15 : ((led < ((scaleA + 1) << 2)) && (led >= (scaleA << 2)) ? 0 : 6);
				monomeLedBuffer[led+64] = !(led & 3) ? 15 : ((led < ((scaleB + 1) << 2)) && (led >= (scaleB << 2)) ? 0 : 6);
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
		if (valueToShow == 1 || arc4index == 1) // divisorArc 0-15
		{
			level = isDivisorArc ? 15 : 0;
			for (u8 led = 0; led < 64; led++)
				monomeLedBuffer[led] = !(led & 3) ? 5 : ((led < ((divisorArc + 1) << 2)) && (led >= (divisorArc << 2)) ? level : 0);
		}
		if (valueToShow == 2 || arc4index == 1) // phaseArc 0-15
		{
			level = isPhaseArc ? 15 : 0;
			for (u8 led = 0; led < 64; led++)
				monomeLedBuffer[led + 64] = !(led & 3) ? 5 : ((led < ((phaseArc + 1) << 2)) && (led >= (phaseArc << 2)) ? level : 0);
		}
		if (valueToShow == 3 || arc4index == 1) // mixerArc 0-15
		{
			level = isMixerArc ? 15 : 0;
			for (u8 led = 0; led < 64; led++)
				monomeLedBuffer[led + 128] = !(led & 3) ? 5 : ((led < ((mixerArc + 1) << 2)) && (led >= (mixerArc << 2)) ? level : 0);
		}
		if (valueToShow == 4 || arc4index == 1) // preset 0-7
		{
			u8 pres = (cb << 3) + banks[cb].cp;
			for (u8 led = 0; led < 64; led++)
				monomeLedBuffer[led + 192] = led == pres ? 15 : (led & 7 ? 1 : 6);
		}
		if (valueToShow == 5) // scale 0-15
		{
			for (u8 led = 0; led < 64; led++)
			{
				monomeLedBuffer[led+64] = !(led & 3) ? 15 : ((led < ((scaleA + 1) << 2)) && (led >= (scaleA << 2)) ? 0 : 6);
				monomeLedBuffer[led+128] = !(led & 3) ? 15 : ((led < ((scaleB + 1) << 2)) && (led >= (scaleB << 2)) ? 0 : 6);
			}
		}
		if (arc4index == 2) // scale page
		{
			for (u8 led = 0; led < 64; led++)
			{
				monomeLedBuffer[led] = monomeLedBuffer[led+192] = 0;
				monomeLedBuffer[led+64] = !(led & 3) ? 15 : ((led < ((scaleA + 1) << 2)) && (led >= (scaleA << 2)) ? 0 : 6);
				monomeLedBuffer[led+128] = !(led & 3) ? 15 : ((led < ((scaleB + 1) << 2)) && (led >= (scaleB << 2)) ? 0 : 6);
			}
		}
	}
	
	if (flashConfirmation == 1)
	{
		for (u8 led = 0; led < 255; led++)
		{
			monomeLedBuffer[led] = 15;
		}
	}
}

void updateCVOutputs(u8 fromClock)
{
	if (gridParam == SCALE && isScalePreview)
	{
		cv0 = cv1 = CHROMATIC[scales[lastSelectedScale ? scaleB : scaleA][(currentScaleRow<<2)+currentScaleColumn]];
	}
	else
	{
		u8 cvA = 0, cvB = 0;
		u16 currentOn, offset;
		
		for (u8 seq = 0; seq < 4; seq++)
		{
			offset = counter[seq] + (divisor[seq] << 6) - phase[seq];
			currentOn = (offset / divisor[seq]) & 1;
			
			if (!fromClock || (chance[seq] < ((rnd() % 20)+1)))
			{
				if ((alwaysOnA & (1 << seq)) | (currentOn && (mixerA & (1 << seq)))) cvA += weight[seq];
				if ((alwaysOnB & (1 << seq)) | (currentOn && (mixerB & (1 << seq)))) cvB += weight[seq]; 
			}
		}
	
		cv0 = CHROMATIC[scales[scaleA][cvA & 0xf]];
		cv1 = CHROMATIC[scales[scaleB][cvB & 0xf]];
	}

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

void updateTriggerOutputs(u8 fromClock)
{
	if (gridParam == SCALE && isScalePreview)
	{
        timer_remove(&triggerSettingsTimerG);
        for (u8 trig = 0; trig < 4; trig++)
        {
            timer_remove(&triggerTimer[trig]);
            timer_remove(&triggerSettingsTimer[trig]);
            if (gateMuted[trig]) gpio_clr_gpio_pin(TRIGGERS[trig]); else gpio_set_gpio_pin(TRIGGERS[trig]);
			timer_add(&triggerTimer[trig], 20, triggerTimer_callbacks[trig], NULL);
        }
        return;
	}
	
	// if only one clock advanced we only want to update triggers that include it
	// if global clock advanced we update all triggers
	// if not from a clock we only update mutes and gates
	
    u16 currentOn[4], offset;
	for (u8 seq = 0; seq < 4; seq++)
	{
		offset = counter[seq] + (divisor[seq] << 6) - phase[seq];
		currentOn[seq] = (offset / divisor[seq]) & 1;
	}
		
	u8 t, trackIncluded;
    u16 trigOn;
	for (u8 trig = 0; trig < 4; trig++)
	{
		t = gateLogic[trig] && gateTracks[trig];
		for (u8 seq = 0; seq < 4; seq++)
		{
			trackIncluded = gateTracks[trig] & (1 << seq);
			trigOn = !gateType[trig] ? prevOn[seq] != currentOn[seq] : !prevOn[seq] && currentOn[seq];
			if (gateLogic[trig]) // AND
			{
				if (trackIncluded && (!fromClock || chance[seq] < ((rnd() % 20)+1))) t &= gateType[trig] == 2 ? currentOn[seq] : trigOn;
			}
			else // OR
			{
				if (trackIncluded && (!fromClock || chance[seq] < ((rnd() % 20)+1))) t |= gateType[trig] == 2 ? currentOn[seq] : trigOn;
			}
		}
		if (gateNot[0]) t = !t;
		
		if (gateType[trig] != 2 && t)
		{
			showTriggers[trig] = 1;
			timer_add(&triggerSettingsTimer[trig], 100, triggerSettingsTimer_callbacks[trig], NULL);
		}
		
		if (gateMuted[trig])
		{
			triggerOn[trig] = 0;
            timer_remove(&triggerTimer[trig]);
			gpio_clr_gpio_pin(TRIGGERS[trig]);
		}
		else if (gateType[trig] == 2)
		{
			triggerOn[trig] = 0;
            timer_remove(&triggerTimer[trig]);
			if (t) gpio_set_gpio_pin(TRIGGERS[trig]); else gpio_clr_gpio_pin(TRIGGERS[trig]);
		}
		else if (t && fromClock && (fromClock == 5 || (gateTracks[trig] & (1 << (fromClock - 1)))))
		{
			triggerOn[trig] = 1;
            timer_add(&triggerTimer[trig], 20, triggerTimer_callbacks[trig], NULL);
            gpio_set_gpio_pin(TRIGGERS[trig]);
		}
	}
	
	for (u8 seq = 0; seq < 4; seq++)
	{
		if (fromClock == 5 || fromClock == seq + 1) prevOn[seq] = currentOn[seq];
	}
}

u8 random8(void)
{
	return (u8)((rnd() ^ adc[1]) & 0xff);
}

u8 isTrackDead(u8 divisor, u8 phase, u8 reset, u8 globalReset)
{
	u8 res = min(reset, globalReset);
	if (!res) res = max(reset, globalReset);
	if (!res) return 0;

	u8 prevValue = (((divisor << 6) - phase) / divisor) & 1, value, dead = 1;
	
	for (u8 i = 1; i < res; i++)
	{
		value = (((divisor << 6) - phase + i) / divisor) & 1;
		if (value != prevValue)
		{
			dead = 0;
			break;
		}
		prevValue = value;
	}
		
	return dead;
}

void generateChaos(void)
{
	for (u8 i = 0; i < 4; i++)
	{
		for (u8 j = 0; j < 16; j++) banks[cb].presets[banks[cb].cp].scales[banks[cb].presets[banks[cb].cp].scaleA][j] = random8() % NOTEMAX;
		for (u8 j = 0; j < 16; j++) banks[cb].presets[banks[cb].cp].scales[banks[cb].presets[banks[cb].cp].scaleB][j] = random8() % NOTEMAX;
		banks[cb].presets[banks[cb].cp].divisor[i] = (random8() & 15) + 1;
		banks[cb].presets[banks[cb].cp].phase[i] = random8() % 16;
		banks[cb].presets[banks[cb].cp].reset[i] = random8() % 16;
		banks[cb].presets[banks[cb].cp].weight[i] = (random8() & 7) + 1;
		banks[cb].presets[banks[cb].cp].gateType[i] = random8() % 3;
		banks[cb].presets[banks[cb].cp].gateLogic[i] = random8() & 1;
		banks[cb].presets[banks[cb].cp].gateNot[i] = random8() & 1;
		banks[cb].presets[banks[cb].cp].gateTracks[i] = random8() & 15;
	}
	banks[cb].presets[banks[cb].cp].mixerA = random8() & 15;
	banks[cb].presets[banks[cb].cp].mixerB = random8() & 15;
	updatePresetCache();
}

void generateRandom(u8 max, u8 paramCount)
{
	for (u8 t = 0; t < 40; t++)
	{
		u8 index = random8() & 3;
		u8 div = divisor[index], ph = phase[index], res = reset[index];
		for (u8 i = 0; i < paramCount; i++)
			switch (random8() % 3)
			{
				case 0:
					div = (random8() % max) + 1;
					break;
				case 1:
					ph = random8() % (max + 1);
					break;
				case 2:
					res = random8() > 128 ? 0 : random8() % (max + 1);
					break;
			}
		if (!isTrackDead(div, ph, res, globalReset))
		{
			banks[cb].presets[banks[cb].cp].divisor[index] = divisor[index] = div;
			banks[cb].presets[banks[cb].cp].phase[index] = phase[index] = ph;
			banks[cb].presets[banks[cb].cp].reset[index] = reset[index] = res;
			break;
		}
	}
	
	/*
	if (paramCount > 2)
	{
		for (u8 i = 0; i < 4; i++)
			banks[cb].presets[banks[cb].cp].weight[i] = weight[i] = (random8() & 7) + 1;
	}

	if (paramCount > 3)
		rotateScales(random8() % max);
	*/
}

void mutate(void)
{
	for (u8 i = 0; i < 40; i++)
	{
		u8 index = random8() & 3;
		u8 div = divisor[index], ph = phase[index], res = reset[index];
		
		switch (random8() % 3)
		{
			case 0:
				div = divisor[index];
				if (random8() & 1)
				{
					if (div > 1) div--;
				}
				else
				{
					if (div < 16) div++;
				}
				break;
			case 1:
				ph = phase[index];
				if (random8() & 1)
				{
					if (ph > 0) ph--;
				}
				else
				{
					if (ph < 16) ph++;
				}
				break;
			case 2:
				res = reset[index];
				if (random8() & 1)
				{
					if (res > 0) res--;
				}
				else
				{
					if (res < 16) res++;
				}
				break;
		}
		
		if (!isTrackDead(div, ph, res, globalReset))
		{
			banks[cb].presets[banks[cb].cp].divisor[index] = divisor[index] = div;
			banks[cb].presets[banks[cb].cp].phase[index] = phase[index] = ph;
			banks[cb].presets[banks[cb].cp].reset[index] = reset[index] = res;
			break;
		}
	}
}

void rotateScales(s8 amount)
{
	if (amount < 0)
	{
		for (s8 j = 0; j < -amount; j++)
		{
			u8 lastScale = scales[scaleA][15];
			scales[scaleA][15] = scales[scaleA][14];
			scales[scaleA][14] = scales[scaleA][13];
			scales[scaleA][13] = scales[scaleA][12];
			scales[scaleA][12] = scales[scaleA][11];
			scales[scaleA][11] = scales[scaleA][10];
			scales[scaleA][10] = scales[scaleA][9];
			scales[scaleA][9] = scales[scaleA][8];
			scales[scaleA][8] = scales[scaleA][7];
			scales[scaleA][7] = scales[scaleA][6];
			scales[scaleA][6] = scales[scaleA][5];
			scales[scaleA][5] = scales[scaleA][4];
			scales[scaleA][4] = scales[scaleA][3];
			scales[scaleA][3] = scales[scaleA][2];
			scales[scaleA][2] = scales[scaleA][1];
			scales[scaleA][1] = scales[scaleA][0];
			scales[scaleA][0] = lastScale;

			lastScale = scales[scaleB][15];
			scales[scaleB][15] = scales[scaleB][14];
			scales[scaleB][14] = scales[scaleB][13];
			scales[scaleB][13] = scales[scaleB][12];
			scales[scaleB][12] = scales[scaleB][11];
			scales[scaleB][11] = scales[scaleB][10];
			scales[scaleB][10] = scales[scaleB][9];
			scales[scaleB][9] = scales[scaleB][8];
			scales[scaleB][8] = scales[scaleB][7];
			scales[scaleB][7] = scales[scaleB][6];
			scales[scaleB][6] = scales[scaleB][5];
			scales[scaleB][5] = scales[scaleB][4];
			scales[scaleB][4] = scales[scaleB][3];
			scales[scaleB][3] = scales[scaleB][2];
			scales[scaleB][2] = scales[scaleB][1];
			scales[scaleB][1] = scales[scaleB][0];
			scales[scaleB][0] = lastScale;
		}
		for (u8 i = 0; i < 8; i++) for (u8 j = 0; j < 8; j++)
			banks[cb].presets[banks[cb].cp].scales[i][j] = scales[i][j];
	} 
	else
	{
		for (s8 j = 0; j < amount; j++)
		{
			u8 firstScale = scales[scaleA][0];
			scales[scaleA][0] = scales[scaleA][1];
			scales[scaleA][1] = scales[scaleA][2];
			scales[scaleA][2] = scales[scaleA][3];
			scales[scaleA][3] = scales[scaleA][4];
			scales[scaleA][4] = scales[scaleA][5];
			scales[scaleA][5] = scales[scaleA][6];
			scales[scaleA][6] = scales[scaleA][7];
			scales[scaleA][7] = scales[scaleA][8];
			scales[scaleA][8] = scales[scaleA][9];
			scales[scaleA][9] = scales[scaleA][10];
			scales[scaleA][10] = scales[scaleA][11];
			scales[scaleA][11] = scales[scaleA][12];
			scales[scaleA][12] = scales[scaleA][13];
			scales[scaleA][13] = scales[scaleA][14];
			scales[scaleA][14] = scales[scaleA][15];
			scales[scaleA][15] = firstScale;

			firstScale = scales[scaleB][0];
			scales[scaleB][0] = scales[scaleB][1];
			scales[scaleB][1] = scales[scaleB][2];
			scales[scaleB][2] = scales[scaleB][3];
			scales[scaleB][3] = scales[scaleB][4];
			scales[scaleB][4] = scales[scaleB][5];
			scales[scaleB][5] = scales[scaleB][6];
			scales[scaleB][6] = scales[scaleB][7];
			scales[scaleB][7] = scales[scaleB][8];
			scales[scaleB][8] = scales[scaleB][9];
			scales[scaleB][9] = scales[scaleB][10];
			scales[scaleB][10] = scales[scaleB][11];
			scales[scaleB][11] = scales[scaleB][12];
			scales[scaleB][12] = scales[scaleB][13];
			scales[scaleB][13] = scales[scaleB][14];
			scales[scaleB][14] = scales[scaleB][15];
			scales[scaleB][15] = firstScale;
		}
		for (u8 i = 0; i < 8; i++) for (u8 j = 0; j < 8; j++)
			banks[cb].presets[banks[cb].cp].scales[i][j] = scales[i][j];
	}
}

void rotateWeights_(s8 amount)
{
	if (amount < 0)
	{
		for (s8 j = 0; j < -amount; j++)
		{
			u8 temp = weight[3];
			weight[3] = weight[2];
			weight[2] = weight[1];
			weight[1] = weight[0];
			weight[0] = temp;
		}
		for (u8 i = 0; i < 4; i++)
			banks[cb].presets[banks[cb].cp].weight[i] = weight[i];
	}

	if (amount > 0)
	{
		for (s8 j = 0; j < amount; j++)
		{
			u8 temp = weight[0];
			weight[0] = weight[1];
			weight[1] = weight[2];
			weight[2] = weight[3];
			weight[3] = temp;
		}
		for (u8 i = 0; i < 4; i++)
			banks[cb].presets[banks[cb].cp].weight[i] = weight[i];
	}
}

void updateglobalLength()
{
	globalLength = 1;
	for (u8 i = 0; i < 4; i++)
		globalLength *= reset[i] ? reset[i] : divisor[i] << 6;
	globalLength <<= 6; // this is for rotate and seq options so that we have at least 64 steps
	globalCounter = globalCounter % globalLength;
}

void adjustCounter(u8 index)
{
	updateglobalLength();
	counter[index] = globalCounter % (reset[index] ? reset[index] : divisor[index] << 6);
}

void adjustAllCounters()
{
	updateglobalLength();
	counter[0] = globalCounter % (reset[0] ? reset[0] : divisor[0] << 6);
	counter[1] = globalCounter % (reset[1] ? reset[1] : divisor[1] << 6);
	counter[2] = globalCounter % (reset[2] ? reset[2] : divisor[2] << 6);
	counter[3] = globalCounter % (reset[3] ? reset[3] : divisor[3] << 6);
}

void clock(u8 phase) {
	u8 seq16;
	
	if (phase)
	{
		gpio_set_gpio_pin(B10);

		globalCounter++;
		if (globalCounter >= globalLength || (globalReset && globalCounter >= globalReset))
		{
			globalCounter = counter[0] = counter[1] = counter[2] = counter[3] = 0;
		}
		else
		{
			for (u8 seq = 0; seq < 4; seq++)
			{
				counter[seq]++;
				if (reset[seq])
				{
					if (counter[seq] >= reset[seq]) counter[seq] = 0;
				}
				else
				{
					if (counter[seq] >= ((u16)divisor[seq] << 6)) counter[seq] = 0;
				}
			}
		}
		
		seq16 = globalCounter & 15;
		
		if (!(gridParam == SCALE && isScalePreview))
			rotateScales(rotateScale[seq16]);

		if (!(gridParam == SCALE && isScalePreview))
			rotateWeights_(rotateWeights[seq16]);
		
		if (!(gridParam == SCALE && isScalePreview) && getMutateSeq(globalCounter & 63))
		{
			mutate();
			adjustAllCounters();
		}
		
		showTriggersG = 1;
		timer_add(&triggerSettingsTimerG, 100, &triggerSettingsTimerG_callback, NULL);
		
		updateCVOutputs(5);
		updateTriggerOutputs(5);
		redraw();
	}
	else {
		gpio_clr_gpio_pin(B10);
 	}
}

void determineArcSets()
{
	isDivisorArc = 1;
	for (u8 i = 0; i < 4; i++)
		if (banks[cb].presets[banks[cb].cp].divisor[i] != divisor_presets[banks[cb].presets[banks[cb].cp].divisorArc][i]) 
			isDivisorArc = 0;
	
	isPhaseArc = 1;
	for (u8 i = 0; i < 4; i++)
		if (banks[cb].presets[banks[cb].cp].phase[i] != phase_presets[banks[cb].presets[banks[cb].cp].phaseArc][i]) 
			isPhaseArc = 0;
		
	isMixerArc = 1;
	if (banks[cb].presets[banks[cb].cp].mixerA != mixer_presets[banks[cb].presets[banks[cb].cp].mixerArc][0]) 
		isMixerArc = 0;
	if (banks[cb].presets[banks[cb].cp].mixerB != mixer_presets[banks[cb].presets[banks[cb].cp].mixerArc][1]) 
		isMixerArc = 0;
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

static void triggerTimer0_callback(void* o) {
	timer_remove(&triggerTimer[0]);
	if (gateType[0] != 2)
    {
        gpio_clr_gpio_pin(TRIGGERS[0]);
        triggerOn[0] = 0;
    }
}

static void triggerTimer1_callback(void* o) {
	timer_remove(&triggerTimer[1]);
	if (gateType[1] != 2)
    {
        gpio_clr_gpio_pin(TRIGGERS[1]);
        triggerOn[1] = 0;
    }
}

static void triggerTimer2_callback(void* o) {
	timer_remove(&triggerTimer[2]);
	if (gateType[2] != 2)
    {
        gpio_clr_gpio_pin(TRIGGERS[2]);
        triggerOn[2] = 0;
    }
}

static void triggerTimer3_callback(void* o) {
	timer_remove(&triggerTimer[3]);
	if (gateType[3] != 2)
    {
        gpio_clr_gpio_pin(TRIGGERS[3]);
        triggerOn[3] = 0;
    }
}

static void triggerSettingsTimer0_callback(void* o) {
	timer_remove(&triggerSettingsTimer[0]);
	showTriggers[0] = 0;
	redraw();
}

static void triggerSettingsTimer1_callback(void* o) {
	timer_remove(&triggerSettingsTimer[1]);
	showTriggers[1] = 0;
	redraw();
}

static void triggerSettingsTimer2_callback(void* o) {
	timer_remove(&triggerSettingsTimer[2]);
	showTriggers[2] = 0;
	redraw();
}

static void triggerSettingsTimer3_callback(void* o) {
	timer_remove(&triggerSettingsTimer[3]);
	showTriggers[3] = 0;
	redraw();
}

static void triggerSettingsTimerG_callback(void* o) {
	timer_remove(&triggerSettingsTimerG);
	showTriggersG = 0;
	redraw();
}

static void cvPreviewTimer_callback(void* o) {
	scaleBlink = ~scaleBlink;
}

static void confirmationTimer_callback(void* o) {
	timer_remove(&confirmationTimer);
	showRandomized = 0;
	presetToShow = bankToShow = 8;
	redraw();
}

static void flashSavedTimer_callback(void* o) {
	flashConfirmation = 0;
	timer_remove(&flashSavedTimer);
	redraw();
}

static void doublePressTimer_callback(void* o)
{
	timer_remove(&doublePressTimer);
	prevXPressed = prevXReleased = 16;
	prevYPressed = prevYReleased = 8;
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
	isArc2 = monome_encs() == 2;
	isVb = monome_is_vari();
	if (monome_device() == eDeviceArc) determineArcSets();
}

static void handler_MonomePoll(s32 data) { monome_read_serial(); }
static void handler_MonomeRefresh(s32 data) {
	if(monomeFrameDirty) {
		(*monome_refresh)();
	}
}

static void showValueTimer_callback(void* o) {
	valueToShow = 0;
	frontClicked = 0;
	timer_remove(&showValueTimer);
	redraw();
}

void showValue(u8 value)
{
	timer_remove(&showValueTimer);
	valueToShow = value;
	timer_add(&showValueTimer, 1000, &showValueTimer_callback, NULL);
	redraw();
}

static void handler_Front(s32 data) {
	if(data == 0) {
		front_timer = 15;
		
		if (isArc)
		{
			if (isArc2)
			{
				if (++arc2index == 3) arc2index = 0;
			}
			else
			{
				if (++arc4index == 3) arc4index = 0;
			}
		}
		
        if (frontClicked) // double click
        {
            for (u8 i = 0; i < 16; i++)
            {
                banks[cb].presets[banks[cb].cp].rotateScale[i] = rotateScale[i] = 0;
                banks[cb].presets[banks[cb].cp].rotateWeights[i] = rotateWeights[i] = 0;
            }
            for (u8 i = 0; i < 64; i++) clrMutateSeq(i);
            for (u8 i = 0; i < 4; i++) banks[cb].presets[banks[cb].cp].chance[i] = chance[i] = banks[cb].presets[banks[cb].cp].reset[i] = reset[i] = 0;
            banks[cb].presets[banks[cb].cp].globalReset = globalReset = globalCounter = counter[0] = counter[1] = counter[2] = counter[3] = 0;
            updateglobalLength();
			frontClicked = 0;
            redraw();
        }
        else
        {
			frontClicked = 1;
			if (!isArc) gridParam = COUNTERS;
            showValue(0);
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
	/*
	u8 newPotValue = (adc[1] >> 8) & 15; // should be in the range 0-15
	if (prevPotValue == 16) // haven't read the value yet
	{
		prevPotValue = newPotValue;
	}
	else if (newPotValue != prevPotValue)
	{
		prevPotValue = scaleA = scaleB = banks[cb].presets[banks[cb].cp].scaleA = banks[cb].presets[banks[cb].cp].scaleB = newPotValue;
		showValue(5);
	}
	*/
}

static void handler_SaveFlash(s32 data) {
	flash_write();
	usb_stick(0);
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

static void orca_process_ii(uint8_t i, int d)
{
    u8 ii;
    int _d;
    
	switch(i)
	{
		case ORCA_TRACK:
			iiTrack = abs(d - 1) & 3;
			break;
		
		case ORCA_DIVISOR:
			divisor[iiTrack] = banks[cb].presets[banks[cb].cp].divisor[iiTrack] = (abs(d - 1) & 15) + 1;
			adjustCounter(iiTrack);
			redraw();
			break;
		
		case ORCA_PHASE:
			phase[iiTrack] = banks[cb].presets[banks[cb].cp].phase[iiTrack] = abs(d) % 17;
			redraw();
			break;
		
		case ORCA_WEIGHT:
			weight[iiTrack] = banks[cb].presets[banks[cb].cp].weight[iiTrack] = (abs(d - 1) & 7) + 1;
			redraw();
			break;
		
		case ORCA_MUTE:
			gateMuted[iiTrack] = banks[cb].presets[banks[cb].cp].gateMuted[iiTrack] = d ? !0 : 0;
			updateTriggerOutputs(0);
			redraw();
			break;
		
		case ORCA_SCALE:
			scaleA = banks[cb].presets[banks[cb].cp].scaleA = (d / 100 - 1) & 15;
			scaleB = banks[cb].presets[banks[cb].cp].scaleB = ((d % 100) - 1) & 15;
			updateCVOutputs(0);
			redraw();
			break;
		
		case ORCA_BANK:
			cb = abs(d - 1) & 7;
			bankToShow = cb;
			updatePresetCache();
			updateCVOutputs(0);
            updateTriggerOutputs(0);
			redraw();
			break;
		
		case ORCA_PRESET:
			banks[cb].cp = abs(d - 1) & 7;
			presetToShow = banks[cb].cp;
			updatePresetCache();
			updateCVOutputs(0);
            updateTriggerOutputs(0);
			redraw();
			break;
			
		case ORCA_RELOAD:
			if (d == 0) // reload current preset
			{
				loadPreset(cb, banks[cb].cp);
				presetToShow = banks[cb].cp;
				updatePresetCache();
			}
			else if (d == 1) // reload current bank
			{
				loadBank(cb, 0);
				bankToShow = cb;
				updatePresetCache();
			}
			else if (d == 2) // reload all banks
			{
				for (u8 b = 0; b < 8; b++) loadBank(b, 0);
				updatePresetCache();
			}
			updateCVOutputs(0);
            updateTriggerOutputs(0);
			redraw();
			break;
		
		case ORCA_ROTATES:
			rotateScales(d % 16);
			updateCVOutputs(0);
			redraw();
			break;
		
		case ORCA_ROTATEW:
			rotateWeights_(d % 4);
			updateCVOutputs(0);
			redraw();
			break;
		
		case ORCA_CVA:
			if (d >= 0)
			{
				mixerA = banks[cb].presets[banks[cb].cp].mixerA = d & 15;
			}
			else
			{
				_d = -d;
				mixerA = alwaysOnA = 0;
				if ((_d / 1000) % 10 == 1) mixerA = 0b1000; else if ((_d / 1000) % 10 == 2) alwaysOnA = 0b1000;
				if ((_d / 100 ) % 10 == 1) mixerA |= 0b0100; else if ((_d / 100) % 10 == 2) alwaysOnA |= 0b0100;
				if ((_d / 10 ) % 10 == 1) mixerA |= 0b0010; else if ((_d / 10) % 10 == 2) alwaysOnA |= 0b0010;
				if (_d % 10 == 1) mixerA |= 0b0001; else if (_d % 10 == 2) alwaysOnA |= 0b0001;
				banks[cb].presets[banks[cb].cp].mixerA = mixerA;
				banks[cb].presets[banks[cb].cp].alwaysOnA = alwaysOnA;
			}
			updateCVOutputs(0);
			redraw();
			break;
		
		case ORCA_CVB:
			if (d >= 0)
			{
				mixerB = banks[cb].presets[banks[cb].cp].mixerB = d & 15;
			}
			else
			{
				_d = -d;
				mixerB = alwaysOnB = 0;
				if ((_d / 1000) % 10 == 1) mixerB = 0b1000; else if ((_d / 1000) % 10 == 2) alwaysOnB = 0b1000;
				if ((_d / 100 ) % 10 == 1) mixerB |= 0b0100; else if ((_d / 100) % 10 == 2) alwaysOnB |= 0b0100;
				if ((_d / 10 ) % 10 == 1) mixerB |= 0b0010; else if ((_d / 10) % 10 == 2) alwaysOnB |= 0b0010;
				if (_d % 10 == 1) mixerB |= 0b0001; else if (_d % 10 == 2) alwaysOnB |= 0b0001;
				banks[cb].presets[banks[cb].cp].mixerB = mixerB;
				banks[cb].presets[banks[cb].cp].alwaysOnB = alwaysOnB;
			}
			updateCVOutputs(0);
			redraw();
			break;
		
		case ORCA_GRESET:
			globalCounter = counter[0] = counter[1] = counter[2] = counter[3] = 0;
			updateCVOutputs(5);
            updateTriggerOutputs(5);
			redraw();
			break;

		case ORCA_CLOCK:
            ii = abs(d - 1) & 3;
			counter[ii]++;
			if (reset[ii])
			{
				if (counter[ii] >= reset[ii]) counter[ii] = 0;
			}
			else
			{
				if (counter[ii] >= ((u16)divisor[ii] << 6)) counter[ii] = 0;
			}
			updateCVOutputs(ii + 1);
            updateTriggerOutputs(ii + 1);
			redraw();
			break;
		
		case ORCA_RESET:
            ii = abs(d - 1) & 3;
			counter[ii] = 0;
			updateCVOutputs(ii + 1);
            updateTriggerOutputs(ii + 1);
			redraw();
			break;
	}
}

////////////////////////////////////////////////////////////////////////////////
// application arc/grid code

static void handler_MonomeRingEnc(s32 data) { 
	u8 n;
	s8 delta;
	monome_ring_enc_parse_event_data(data, &n, &delta);
	
	if (delta > 0)
	{
		if (encoderDelta[n] > 0) encoderDelta[n] += delta; else encoderDelta[n] = delta;
	}
	else
	{
		if (encoderDelta[n] < 0) encoderDelta[n] += delta; else encoderDelta[n] = delta;
	}
	
	if (abs(encoderDelta[n]) < ENCODER_DELTA_SENSITIVITY)
		return;
	
	encoderDelta[n] = 0;
	
	if (isArc2)
	{
		switch(n)
		{
			case 0:
				if (arc2index == 2) // scale selection page
				{
					if (delta > 0)
					{
						if (++scaleA > 15) scaleA = 0;
					}
					else
					{
						if (scaleA > 0) scaleA--; else scaleA = 15;
					}
					banks[cb].presets[banks[cb].cp].scaleA = scaleA;
					showValue(0);
					return;
				}
				if (delta < 0)
				{
					if (!isDivisorArc)
					{
						isDivisorArc = 1;
						divisorArc = 0;
					}
					else
					{
						if (divisorArc == 0) divisorArc = 15; else divisorArc--;
					}
					banks[cb].presets[banks[cb].cp].divisorArc = divisorArc;
					for (u8 seq = 0; seq < 4; seq++)
					{
						banks[cb].presets[banks[cb].cp].divisor[seq] = divisor[seq] = divisor_presets[divisorArc][seq];
					}						
					adjustAllCounters();
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
					banks[cb].presets[banks[cb].cp].phaseArc = phaseArc;
					for (u8 seq = 0; seq < 4; seq++) banks[cb].presets[banks[cb].cp].phase[seq] = phase[seq] = phase_presets[phaseArc][seq];
					showValue(2);
				}
				break;
			case 1:
				if (arc2index == 2) // scale selection page
				{
					if (delta > 0)
					{
						if (++scaleB > 15) scaleB = 0;
					}
					else
					{
						if (scaleB > 0) scaleB--; else scaleB = 15;
					}
					banks[cb].presets[banks[cb].cp].scaleB = scaleB;
					showValue(0);
					return;
				}
				if (delta < 0)
				{
					if (!isMixerArc)
					{
						isMixerArc = 1;
						mixerArc = 0;
					} else
					{
						if (mixerArc == 0) mixerArc = 15; else mixerArc--;
					}
					banks[cb].presets[banks[cb].cp].mixerArc = mixerArc;
					banks[cb].presets[banks[cb].cp].mixerA = mixerA = mixer_presets[mixerArc][0];
					banks[cb].presets[banks[cb].cp].mixerB = mixerB = mixer_presets[mixerArc][1];
					showValue(3);
				}
				else
				{
					if (banks[cb].cp < 7)
						banks[cb].cp++;
					else
					{
						if (++cb > 7) cb = 0;
						banks[cb].cp = 0;
					}
					updatePresetCache();
					determineArcSets();
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
				if (arc4index == 2) // scale selection page
				{
					return;
				}
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
				banks[cb].presets[banks[cb].cp].divisorArc = divisorArc;
				for (u8 seq = 0; seq < 4; seq++) banks[cb].presets[banks[cb].cp].divisor[seq] = divisor[seq] = divisor_presets[divisorArc][seq];
				adjustAllCounters();
				showValue(1);
				break;
			case 1:
				if (arc4index == 2) // scale selection page
				{
					if (delta > 0)
					{
						if (++scaleA > 15) scaleA = 0;
					}
					else
					{
						if (scaleA > 0) scaleA--; else scaleA = 15;
					}
					banks[cb].presets[banks[cb].cp].scaleA = scaleA;
					showValue(0);
					return;
				}
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
				banks[cb].presets[banks[cb].cp].phaseArc = phaseArc;
				for (u8 seq = 0; seq < 4; seq++) banks[cb].presets[banks[cb].cp].phase[seq] = phase[seq] = phase_presets[phaseArc][seq];
				showValue(2);
				break;
			case 2:
				if (arc4index == 2) // scale selection page
				{
					if (delta > 0)
					{
						if (++scaleB > 15) scaleB = 0;
					}
					else
					{
						if (scaleB > 0) scaleB--; else scaleB = 15;
					}
					banks[cb].presets[banks[cb].cp].scaleB = scaleB;
					showValue(0);
					return;
				}
				if (!isMixerArc)
				{
					isMixerArc = 1;
					mixerArc = 0;
					banks[cb].presets[banks[cb].cp].alwaysOnA = banks[cb].presets[banks[cb].cp].alwaysOnB = alwaysOnA = alwaysOnB = 0;
				}
				else if (delta > 0)
				{
					if (++mixerArc > 15) mixerArc = 0;
				}
				else
				{
					if (mixerArc > 0) mixerArc--; else mixerArc = 15;
				}
				banks[cb].presets[banks[cb].cp].mixerArc = mixerArc;
				banks[cb].presets[banks[cb].cp].mixerA = mixerA = mixer_presets[mixerArc][0];
				banks[cb].presets[banks[cb].cp].mixerB = mixerB = mixer_presets[mixerArc][1];
				showValue(3);
				break;
			case 3:
				if (arc4index == 2) // scale selection page
				{
					return;
				}
				if (delta > 0)
				{
					if (banks[cb].cp < 7)
						banks[cb].cp++;
					else
					{
						if (++cb > 7) cb = 0;
						banks[cb].cp = 0;
					}
				}
				else
				{
					if (banks[cb].cp > 0)
						banks[cb].cp--;
					else
					{
						if (cb > 0) cb--; else cb = 7;
						banks[cb].cp = 7;
					}
				}			
				updatePresetCache();
				determineArcSets();
				showValue(4);
				break;
		}
	}
}

static void handler_MonomeGridKey(s32 data)
{ 
	u8 x, y, z;
	monome_grid_key_parse_event_data(data, &x, &y, &z);
	// z == 0 key up, z == 1 key down

	timer_remove(&doublePressTimer);
	timer_add(&doublePressTimer, 200, &doublePressTimer_callback, NULL);
	timer_remove(&confirmationTimer);
	timer_add(&confirmationTimer, 100, &confirmationTimer_callback, NULL);

	if (!z) // key up
	{
		prevXReleased = x;
		prevYReleased = y;
		
		if (x == 1 && y == 0)
		{
			if (scalePreviewEnabled) isScalePreview = !isScalePreview;
			scalePreviewEnabled = scalePressed = 0;
		}
	
        if (x == 1 && y == 3)
        {
            presetModePressed = 0;
        }
    
		if (gridParam == SCALE)
		{
			if (y == 4 && scalePresetPressed == x) scalePresetPressed = 16;
			if (y == 6 && userScalePressed == x && !lastSelectedScale) userScalePressed = 16;
			if (y == 7 && userScalePressed == x && lastSelectedScale) userScalePressed = 16;
		}
	
		if (gridParam == PRESETS)
		{
			if (y < 6 && bankPressed == (x >> 1)) bankPressed = 8;
			if (y > 5 && presetPressed == (x >> 1)) presetPressed = 8;
		}
		
		if (notePressedIndex == x - 4 && currentScaleColumn == y - 4)
		{
			notePressedIndex = 12;
		}
		
		if (noteSquarePressedCol == x && noteSquarePressedRow == y - 4)
		{
			noteSquarePressedCol = noteSquarePressedRow = 4;
		}

		return;
	}
	
	if (notePressedIndex < 12 && gridParam == SCALE) // octave selection
	{
        if (x - 4 != notePressedIndex || y - 4 != currentScaleColumn)
		{
			u8 noteIndex = scales[lastSelectedScale ? scaleB : scaleA][(currentScaleRow<<2)+currentScaleColumn]; // currently selected note
			u8 octave = noteIndex / 12; // current octave
			
			if (octave > 0 && (y - 4 > currentScaleColumn || x - 4 < notePressedIndex)) // octave down
				octave--;
			else if (y - 4 < currentScaleColumn || x - 4 > notePressedIndex) // octave up
			{
				octave++;
				if (octave > NOTEMAX / 12 - 1) octave = NOTEMAX / 12 - 1;
			}
			noteIndex = (noteIndex % 12) + octave * 12;
			banks[cb].presets[banks[cb].cp].scales[lastSelectedScale ? scaleB : scaleA][(currentScaleRow<<2)+currentScaleColumn] = scales[lastSelectedScale ? scaleB : scaleA][(currentScaleRow<<2)+currentScaleColumn] = noteIndex;
			if (isScalePreview) updateCVOutputs(0);
			redraw();
			return;
		}
	}
	
	if (noteSquarePressedCol < 4 && gridParam == SCALE) // octave / semitone shift for a full row
	{
		u8 can = 1, ni = noteSquarePressedRow << 2;
		u8 scale = lastSelectedScale ? scaleB : scaleA;
		if (noteSquarePressedCol == x)
		{
			if (y - 4 - noteSquarePressedRow == 2) // octave down
			{
				for (u8 i = 0; i < 4; i++) if (scales[scale][ni+i] < 12) { can = 0; break; }
				if (can) for (u8 i = 0; i < 4; i++) { banks[cb].presets[banks[cb].cp].scales[scale][ni+i] -= 12; scales[scale][ni+i] -= 12; }
			}
			else if (noteSquarePressedRow - y + 4 == 2) // octave up
			{
				for (u8 i = 0; i < 4; i++) if (scales[scale][ni+i] >= NOTEMAX - 12) { can = 0; break; }
				if (can) for (u8 i = 0; i < 4; i++) { banks[cb].presets[banks[cb].cp].scales[scale][ni+i] += 12; scales[scale][ni+i] += 12; }
			}
			else if (y - 4 - noteSquarePressedRow == 1) // semitone down
			{
				for (u8 i = 0; i < 4; i++) if (scales[scale][ni+i] == 0) { can = 0; break; }
				if (can) for (u8 i = 0; i < 4; i++) { banks[cb].presets[banks[cb].cp].scales[scale][ni+i]--; scales[scale][ni+i]--; }
			}
			else if (noteSquarePressedRow - y + 4 == 1) // semitone up
			{
				for (u8 i = 0; i < 4; i++) if (scales[scale][ni+i] == NOTEMAX - 1) { can = 0; break; }
				if (can) for (u8 i = 0; i < 4; i++) { banks[cb].presets[banks[cb].cp].scales[scale][ni+i]++; scales[scale][ni+i]++; }
			}
		}
		else if (noteSquarePressedRow == y - 4)
		{
			if (noteSquarePressedCol - x == 2) // octave down
			{
				for (u8 i = 0; i < 4; i++) if (scales[scale][ni+i] < 12) { can = 0; break; }
				if (can) for (u8 i = 0; i < 4; i++) { banks[cb].presets[banks[cb].cp].scales[scale][ni+i] -= 12; scales[scale][ni+i] -= 12; }
			}
			else if (x - noteSquarePressedCol == 2) // octave up
			{
				for (u8 i = 0; i < 4; i++) if (scales[scale][ni+i] >= NOTEMAX - 12) { can = 0; break; }
				if (can) for (u8 i = 0; i < 4; i++) { banks[cb].presets[banks[cb].cp].scales[scale][ni+i] += 12; scales[scale][ni+i] += 12; }
			}
			else if (noteSquarePressedCol - x == 1) // semitone down
			{
				for (u8 i = 0; i < 4; i++) if (scales[scale][ni+i] == 0) { can = 0; break; }
				if (can) for (u8 i = 0; i < 4; i++) { banks[cb].presets[banks[cb].cp].scales[scale][ni+i]--; scales[scale][ni+i]--; }
			}
			else if (x - noteSquarePressedCol == 1) // semitone up
			{
				for (u8 i = 0; i < 4; i++) if (scales[scale][ni+i] == NOTEMAX - 1) { can = 0; break; }
				if (can) for (u8 i = 0; i < 4; i++) { banks[cb].presets[banks[cb].cp].scales[scale][ni+i]++; scales[scale][ni+i]++; }
			}
		}
		if (isScalePreview) updateCVOutputs(0);
		redraw();
		return;
	}
	
	u8 doublePress = prevXPressed == prevXReleased && prevXReleased == x && prevYPressed == prevYReleased && prevYReleased == y;
	prevXPressed = x;
	prevYPressed = y;
	
	if (x == 0 && y < 4) // select divisor / phase / reset / chance 
	{
		if (y == 0) gridParam = DIVISOR;
		else if (y == 1) gridParam = PHASE;
		else if (y == 2) gridParam = RESET;
		else if (y == 3) gridParam = CHANCE;
		
		redraw();
		return;
	}
	
	if (x == 1 && y < 4) // select scale edit / settings / randomize / presets 
	{
		if (y == 0)
		{
			if (gridParam == SCALE) scalePreviewEnabled = 1;
			gridParam = SCALE;
			scalePressed = 1;
		}
		else if (y == 1) gridParam = SETTINGS;
		else if (y == 2)
		{
			gridParam = RANDOMIZE;
			if (doublePress)
			{
				generateChaos();
				showRandomized = 2;
			}
			adjustAllCounters();
		}
		else if (y == 3)
        {
            gridParam = PRESETS;
            presetModePressed = 1;
            if (doublePress) flash_write();
        }
		
		redraw();
		return;
	}
	
	if (x == 2 && y < 4) // select rotate scale / rotate weights / mutate seq / global reset
	{
		if (y == 0)
		{
			gridParam = ROTATESCALE;
			if (doublePress) for (u8 i = 0; i < 16; i++) banks[cb].presets[banks[cb].cp].rotateScale[i] = rotateScale[i] = 0;
		}
		else if (y == 1)
		{
			gridParam = ROTATEWEIGHTS;
			if (doublePress) for (u8 i = 0; i < 16; i++) banks[cb].presets[banks[cb].cp].rotateWeights[i] = rotateWeights[i] = 0;
		}
		else if (y == 2) 
		{
			gridParam = MUTATESEQ;
			if (doublePress) for (u8 i = 0; i < 64; i++) clrMutateSeq(i);
		}
		else if (y == 3)
		{
			gridParam = GLOBALRESET;
			if (doublePress)
			{
				globalCounter = counter[0] = counter[1] = counter[2] = counter[3] = 0;
			}
		}
		
		redraw();
		return;
	}
	
	if (x == 14 && y < 4) // CV A mixing
	{
		if (gridParam == SCALE)
		{
			if (lastSelectedScale)
			{
				lastSelectedScale = 0;
				redraw();
				return;
			}
		}
		if (doublePress)
		{
			alwaysOnA |= 1 << y;
		}
		else
		{
			alwaysOnA &= ~(1 << y);
			if (mixerA & (1 << y))
				mixerA &= ~(1 << y);
			else
				mixerA |= (1 << y);
			banks[cb].presets[banks[cb].cp].mixerA = mixerA;
		}
		banks[cb].presets[banks[cb].cp].alwaysOnA = alwaysOnA;
		lastSelectedScale = 0;
		redraw();
		return;
	}

	if (x == 15 && y < 4) // CV B mixing
	{
		if (gridParam == SCALE)
		{
			if (!lastSelectedScale)
			{
				lastSelectedScale = 1;
				redraw();
				return;
			}
		}
		if (doublePress)
		{
			alwaysOnB |= 1 << y;
		}
		else
		{
			alwaysOnB &= ~(1 << y);
			if (mixerB & (1 << y))
				mixerB &= ~(1 << y);
			else
				mixerB |= (1 << y);
			banks[cb].presets[banks[cb].cp].mixerB = mixerB;
		}
		banks[cb].presets[banks[cb].cp].alwaysOnB = alwaysOnB;
		lastSelectedScale = 1;
		redraw();
		return;
	}
	
	if (y < 4) return;
	
	// param editing, bottom 4 rows
	
	if (gridParam == SCALE)
	{
		if (scalePressed)
		{
			scalePreviewEnabled = 0;
			
			if (y == 5) // shared scale pressed
			{
				if (scalePresetPressed != 16) // copy from a scale preset to a shared scale
				{
					for (u8 i = 0; i < 16; i++) userScalePresets[x][i] = SCALE_PRESETS[scalePresetPressed][i];
				}
				else if (userScalePressed != 16) // copy from a user scale to a shared scale
				{
					for (u8 i = 0; i < 16; i++) userScalePresets[x][i] = scales[userScalePressed][i];
				}
				else // copy from a shared scale into currently selected user scale
				{
					for (u8 i = 0; i < 16; i++) banks[cb].presets[banks[cb].cp].scales[lastSelectedScale ? scaleB : scaleA][i] = scales[lastSelectedScale ? scaleB : scaleA][i] = userScalePresets[x][i];
				}
			}
			else if (y == 4) // copy from a scale preset into currently selected user scale
			{
				for (u8 i = 0; i < 16; i++) banks[cb].presets[banks[cb].cp].scales[lastSelectedScale ? scaleB : scaleA][i] = scales[lastSelectedScale ? scaleB : scaleA][i] = SCALE_PRESETS[x][i];
				scalePresetPressed = x;
			}
			else
			{
				if (userScalePressed != 16) // copy from one user scale to another
				{
					for (u8 i = 0; i < 16; i++) banks[cb].presets[banks[cb].cp].scales[x][i] = scales[x][i] = banks[cb].presets[banks[cb].cp].scales[userScalePressed][i];
				}
				else // select a user scale
				{
					if (y == 6) // CV A scale
					{
						banks[cb].presets[banks[cb].cp].scaleA = scaleA = x;
						lastSelectedScale = 0;
					}
					else
					{
						banks[cb].presets[banks[cb].cp].scaleB = scaleB = x;
						lastSelectedScale = 1;
					}
					userScalePressed = x;
				}
			}
			redraw();
			return;
		}
		
		if (x < 4)
		{
			noteSquarePressedRow = currentScaleRow = y - 4;
			noteSquarePressedCol = currentScaleColumn = x;
            prevSelectedScaleColumn = 4;
            
			if (isScalePreview) updateCVOutputs(0);
			redraw();
			return;
		}
        
        x -= 4;
        currentScaleColumn = y - 4; // yep, y sets column here as the "piano roll" is pivoted
        u8 noteIndex = scales[lastSelectedScale ? scaleB : scaleA][(currentScaleRow<<2)+currentScaleColumn]; // currently selected note
        
		if (noteIndex % 12 == x) // same note
		{
			// disabling the old way to scroll through octaves
            // if (prevSelectedScaleColumn == currentScaleColumn)
            //    noteIndex = (noteIndex + 12) % NOTEMAX; // raise octave
		}
		else
		{
			noteIndex = x + noteIndex / 12 * 12; // select a different note within the same octave
		}
        prevSelectedScaleColumn = currentScaleColumn;
        
		banks[cb].presets[banks[cb].cp].scales[lastSelectedScale ? scaleB : scaleA][(currentScaleRow<<2)+currentScaleColumn] = scales[lastSelectedScale ? scaleB : scaleA][(currentScaleRow<<2)+currentScaleColumn] = noteIndex;
		
		notePressedIndex = x;
		
		if (isScalePreview) updateCVOutputs(0);
		redraw();
		return;
	}
	else if (gridParam == SETTINGS)
	{
		if (x == 0)
		{
			banks[cb].presets[banks[cb].cp].gateMuted[y - 4] = gateMuted[y - 4] = !gateMuted[y - 4];
		}
		else if (x == 1)
		{
			if (++gateType[y - 4] > 2) gateType[y - 4] = 0;
			banks[cb].presets[banks[cb].cp].gateType[y - 4] = gateType[y - 4];
		}
		else if (x == 2)
		{
			banks[cb].presets[banks[cb].cp].gateLogic[y - 4] = gateLogic[y - 4] = !gateLogic[y - 4];
		}
		else if (x == 3)
		{
			banks[cb].presets[banks[cb].cp].gateNot[y - 4] = gateNot[y - 4] = !gateNot[y - 4];
		}
		else if (x < 8)
		{
			u8 track = 1 << (x - 4);
			if (gateTracks[y - 4] & track)
				gateTracks[y - 4] &= ~track;
			else
				gateTracks[y - 4] |= track;
			banks[cb].presets[banks[cb].cp].gateTracks[y - 4] = gateTracks[y - 4];
		}
		else
		{
			banks[cb].presets[banks[cb].cp].weight[y - 4] = weight[y - 4] = x - 7;
		}
        updateTriggerOutputs(0);
		redraw();
		return;
	}
	else if (gridParam == RANDOMIZE)
	{
		randomizeX = x;
		randomizeY = y - 4;
		generateRandom(randomizeX + 1, randomizeY + 1);
		showRandomized = 1;
		adjustAllCounters();
		redraw();
		return;
	}
    else if (gridParam == PRESETS)
    {
        if (y < 6) // bank selection
        {
			if (bankPressed != 8 && bankPressed != (x >> 1)) // bank copy
			{
				copyBank(bankPressed, x >> 1);
				bankToShow = x >> 1;
			}
			else
			{
				cb = x >> 1;
				bankPressed = cb;
				
				if (doublePress)
				{
					loadBank(cb, 0);
					bankToShow = cb;
				}
				updatePresetCache();
			}
        }
        else // preset selection
        {
			if (presetPressed != 8 && presetPressed != (x >> 1)) // preset copy
			{
				copyPreset(presetPressed, x >> 1);
				presetToShow = x >> 1;
			}
			else
			{
				banks[cb].cp = x >> 1;
				presetPressed = banks[cb].cp;
				
				if (doublePress)
				{
					loadPreset(cb, banks[cb].cp);
					presetToShow = banks[cb].cp;
				}
				updatePresetCache();
			}
        }
        redraw();
        return;
    }
	else if (gridParam == ROTATESCALE)
	{
		s8 value = y < 6 ? 6 - y : 5 - y;
		banks[cb].presets[banks[cb].cp].rotateScale[x] = rotateScale[x] = rotateScale[x] == value ? 0 : value;
		redraw();
		return;
	}
	else if (gridParam == ROTATEWEIGHTS)
	{
		s8 value = y < 6 ? 6 - y : 5 - y;
		banks[cb].presets[banks[cb].cp].rotateWeights[x] = rotateWeights[x] = rotateWeights[x] == value ? 0 : value;
		redraw();
		return;
	}
	else if (gridParam == MUTATESEQ)
	{
		u8 index = ((y - 4) << 4) + x;
		if (getMutateSeq(index)) clrMutateSeq(index); else setMutateSeq(index);
		redraw();
		return;
	}
	else if (gridParam == GLOBALRESET)
	{
		u8 index = ((y - 4) << 4) + x + 1;
		banks[cb].presets[banks[cb].cp].globalReset = globalReset = globalReset == index ? 0 : index;
		redraw();
		return;
	}
	else // param editing
	{
		switch (gridParam)
		{
			case DIVISOR:
				banks[cb].presets[banks[cb].cp].divisor[y - 4] = divisor[y - 4] = x + 1;
				adjustCounter(y - 4);
				break;
			case PHASE:
				if (phase[y - 4] == x + 1)
					phase[y - 4] = 0;
				else
					phase[y - 4] = x + 1;
				banks[cb].presets[banks[cb].cp].phase[y - 4] = phase[y - 4];
				break;
			case RESET:
				if (reset[y - 4] == x + 1)
				{
					reset[y - 4] = 0;
				}
				else
				{
					reset[y - 4] = x + 1;
				}
				banks[cb].presets[banks[cb].cp].reset[y - 4] = reset[y - 4];
				adjustCounter(y - 4);
				break;
			case CHANCE:
				if (chance[y - 4] == x + 1)
					chance[y - 4] = 0;
				else
					chance[y - 4] = x + 1;
				banks[cb].presets[banks[cb].cp].chance[y - 4] = chance[y - 4];
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

void loadPreset(u8 b, u8 p)
{
	banks[b].presets[p].scaleA = flashy.banks[b].presets[p].scaleA;
	banks[b].presets[p].scaleB = flashy.banks[b].presets[p].scaleB;
	for (u8 j = 0; j < 16; j++) for (u8 k = 0; k < 16; k++) banks[b].presets[p].scales[j][k] = flashy.banks[b].presets[p].scales[j][k];
	banks[b].presets[p].divisorArc = flashy.banks[b].presets[p].divisorArc;
	banks[b].presets[p].phaseArc = flashy.banks[b].presets[p].phaseArc;
	banks[b].presets[p].mixerArc = flashy.banks[b].presets[p].mixerArc;
	for (u8 j = 0; j < 4; j++)
	{
		banks[b].presets[p].divisor[j] = flashy.banks[b].presets[p].divisor[j];
		banks[b].presets[p].phase[j] = flashy.banks[b].presets[p].phase[j];
		banks[b].presets[p].reset[j] = flashy.banks[b].presets[p].reset[j];
		banks[b].presets[p].chance[j] = flashy.banks[b].presets[p].chance[j];
		banks[b].presets[p].weight[j] = flashy.banks[b].presets[p].weight[j];
		banks[b].presets[p].gateType[j] = flashy.banks[b].presets[p].gateType[j];
		banks[b].presets[p].gateMuted[j] = flashy.banks[b].presets[p].gateMuted[j];
		banks[b].presets[p].gateLogic[j] = flashy.banks[b].presets[p].gateLogic[j];
		banks[b].presets[p].gateNot[j] = flashy.banks[b].presets[p].gateNot[j];
		banks[b].presets[p].gateTracks[j] = flashy.banks[b].presets[p].gateTracks[j];
	}
	banks[b].presets[p].mixerA = flashy.banks[b].presets[p].mixerA;
	banks[b].presets[p].mixerB = flashy.banks[b].presets[p].mixerB;
	banks[b].presets[p].alwaysOnA = flashy.banks[b].presets[p].alwaysOnA;
	banks[b].presets[p].alwaysOnB = flashy.banks[b].presets[p].alwaysOnB;
	for (u8 k = 0; k < 16; k++) banks[b].presets[p].rotateScale[k] = flashy.banks[b].presets[p].rotateScale[k];
	for (u8 k = 0; k < 16; k++) banks[b].presets[p].rotateWeights[k] = flashy.banks[b].presets[p].rotateWeights[k];
	for (u8 k = 0; k < 8; k++) banks[b].presets[p].mutateSeq[k] = flashy.banks[b].presets[p].mutateSeq[k];
	banks[b].presets[p].globalReset = flashy.banks[b].presets[p].globalReset;
}

void loadBank(u8 b, u8 updatecp)
{
	for (u8 p = 0; p < 8; p++)
	{
		loadPreset(b, p);
	}
	if (updatecp) banks[b].cp = flashy.banks[b].cp;
}

void copyPreset(u8 source, u8 dest)
{
	banks[cb].presets[dest].scaleA = banks[cb].presets[source].scaleA;
	banks[cb].presets[dest].scaleB = banks[cb].presets[source].scaleB;
	for (u8 j = 0; j < 16; j++) for (u8 k = 0; k < 16; k++) banks[cb].presets[dest].scales[j][k] = banks[cb].presets[source].scales[j][k];
	banks[cb].presets[dest].divisorArc = banks[cb].presets[source].divisorArc;
	banks[cb].presets[dest].phaseArc = banks[cb].presets[source].phaseArc;
	banks[cb].presets[dest].mixerArc = banks[cb].presets[source].mixerArc;
	for (u8 j = 0; j < 4; j++)
	{
		banks[cb].presets[dest].divisor[j] = banks[cb].presets[source].divisor[j];
		banks[cb].presets[dest].phase[j] = banks[cb].presets[source].phase[j];
		banks[cb].presets[dest].reset[j] = banks[cb].presets[source].reset[j];
		banks[cb].presets[dest].chance[j] = banks[cb].presets[source].chance[j];
		banks[cb].presets[dest].weight[j] = banks[cb].presets[source].weight[j];
		banks[cb].presets[dest].gateType[j] = banks[cb].presets[source].gateType[j];
		banks[cb].presets[dest].gateMuted[j] = banks[cb].presets[source].gateMuted[j];
		banks[cb].presets[dest].gateLogic[j] = banks[cb].presets[source].gateLogic[j];
		banks[cb].presets[dest].gateNot[j] = banks[cb].presets[source].gateNot[j];
		banks[cb].presets[dest].gateTracks[j] = banks[cb].presets[source].gateTracks[j];
	}
	banks[cb].presets[dest].mixerA = banks[cb].presets[source].mixerA;
	banks[cb].presets[dest].mixerB = banks[cb].presets[source].mixerB;
	banks[cb].presets[dest].alwaysOnA = banks[cb].presets[source].alwaysOnA;
	banks[cb].presets[dest].alwaysOnB = banks[cb].presets[source].alwaysOnB;
	for (u8 k = 0; k < 16; k++) banks[cb].presets[dest].rotateScale[k] = banks[cb].presets[source].rotateScale[k];
	for (u8 k = 0; k < 16; k++) banks[cb].presets[dest].rotateWeights[k] = banks[cb].presets[source].rotateWeights[k];
	for (u8 k = 0; k < 8; k++) banks[cb].presets[dest].mutateSeq[k] = banks[cb].presets[source].mutateSeq[k];
	banks[cb].presets[dest].globalReset = banks[cb].presets[source].globalReset;
}

void copyBank(u8 source, u8 dest)
{
	banks[dest].cp = banks[source].cp;
	for (u8 i = 0; i < 8; i++)
	{
		banks[dest].presets[i].scaleA = banks[source].presets[i].scaleA;
		banks[dest].presets[i].scaleB = banks[source].presets[i].scaleB;
		for (u8 j = 0; j < 16; j++) for (u8 k = 0; k < 16; k++) banks[dest].presets[i].scales[j][k] = banks[source].presets[i].scales[j][k];
		banks[dest].presets[i].divisorArc = banks[source].presets[i].divisorArc;
		banks[dest].presets[i].phaseArc = banks[source].presets[i].phaseArc;
		banks[dest].presets[i].mixerArc = banks[source].presets[i].mixerArc;
		for (u8 j = 0; j < 4; j++)
		{
			banks[dest].presets[i].divisor[j] = banks[source].presets[i].divisor[j];
			banks[dest].presets[i].phase[j] = banks[source].presets[i].phase[j];
			banks[dest].presets[i].reset[j] = banks[source].presets[i].reset[j];
			banks[dest].presets[i].chance[j] = banks[source].presets[i].chance[j];
			banks[dest].presets[i].weight[j] = banks[source].presets[i].weight[j];
			banks[dest].presets[i].gateType[j] = banks[source].presets[i].gateType[j];
			banks[dest].presets[i].gateMuted[j] = banks[source].presets[i].gateMuted[j];
			banks[dest].presets[i].gateLogic[j] = banks[source].presets[i].gateLogic[j];
			banks[dest].presets[i].gateNot[j] = banks[source].presets[i].gateNot[j];
			banks[dest].presets[i].gateTracks[j] = banks[source].presets[i].gateTracks[j];
		}
		banks[dest].presets[i].mixerA = banks[source].presets[i].mixerA;
		banks[dest].presets[i].mixerB = banks[source].presets[i].mixerB;
		banks[dest].presets[i].alwaysOnA = banks[source].presets[i].alwaysOnA;
		banks[dest].presets[i].alwaysOnB = banks[source].presets[i].alwaysOnB;
		for (u8 k = 0; k < 16; k++) banks[dest].presets[i].rotateScale[k] = banks[source].presets[i].rotateScale[k];
		for (u8 k = 0; k < 16; k++) banks[dest].presets[i].rotateWeights[k] = banks[source].presets[i].rotateWeights[k];
		for (u8 k = 0; k < 8; k++) banks[dest].presets[i].mutateSeq[k] = banks[source].presets[i].mutateSeq[k];
		banks[dest].presets[i].globalReset = banks[source].presets[i].globalReset;
	}
}

void updatePresetCache(void)
{
	scaleA = banks[cb].presets[banks[cb].cp].scaleA;
	scaleB = banks[cb].presets[banks[cb].cp].scaleB;
	for (u8 j = 0; j < 16; j++) for (u8 k = 0; k < 16; k++) scales[j][k] = banks[cb].presets[banks[cb].cp].scales[j][k];
	divisorArc = banks[cb].presets[banks[cb].cp].divisorArc;
	phaseArc = banks[cb].presets[banks[cb].cp].phaseArc;
	mixerArc = banks[cb].presets[banks[cb].cp].mixerArc;
	for (u8 j = 0; j < 4; j++)
	{
		divisor[j] = banks[cb].presets[banks[cb].cp].divisor[j];
		phase[j] = banks[cb].presets[banks[cb].cp].phase[j];
		reset[j] = banks[cb].presets[banks[cb].cp].reset[j];
		chance[j] = banks[cb].presets[banks[cb].cp].chance[j];
		weight[j] = banks[cb].presets[banks[cb].cp].weight[j];
		gateType[j] = banks[cb].presets[banks[cb].cp].gateType[j];
		gateMuted[j] = banks[cb].presets[banks[cb].cp].gateMuted[j];
		gateLogic[j] = banks[cb].presets[banks[cb].cp].gateLogic[j];
		gateNot[j] = banks[cb].presets[banks[cb].cp].gateNot[j];
		gateTracks[j] = banks[cb].presets[banks[cb].cp].gateTracks[j];
	}
	mixerA = banks[cb].presets[banks[cb].cp].mixerA;
	mixerB = banks[cb].presets[banks[cb].cp].mixerB;
	alwaysOnA = banks[cb].presets[banks[cb].cp].alwaysOnA;
	alwaysOnB = banks[cb].presets[banks[cb].cp].alwaysOnB;
	for (u8 k = 0; k < 16; k++) rotateScale[k] = banks[cb].presets[banks[cb].cp].rotateScale[k];
	for (u8 k = 0; k < 16; k++) rotateWeights[k] = banks[cb].presets[banks[cb].cp].rotateWeights[k];
	for (u8 k = 0; k < 8; k++) mutateSeq[k] = banks[cb].presets[banks[cb].cp].mutateSeq[k];
	globalReset = banks[cb].presets[banks[cb].cp].globalReset;
	adjustAllCounters();
}

void usb_write_str(char* str)
{
	file_write_buf((uint8_t*) str, strlen(str));
	file_putc('\n');
}

void usb_write_param(char* name, u16 value, int base)
{
	file_write_buf((uint8_t*) name, strlen(name));
	file_putc(':');	file_putc(' ');
	itoa(value, str, base);
	if (base == 2)
	{
		for (s8 i = strlen(str) - 1; i >= 0; i--) file_putc(str[i]);
		for (s8 i = 0; i < 4 - strlen(str); i++) file_putc('0');
	}
	else
	{
		file_write_buf((uint8_t*) str, strlen(str));
	}
	file_putc('\n');
}

void usb_write_u8_array(char* name, u8* array, u8 length, int base, u8 padding)
{
	file_write_buf((uint8_t*) name, strlen(name));
	file_putc(':');	file_putc(' ');
	for (u8 i = 0; i < length; i++)
	{
		itoa(array[i], str, base);
		if (base == 2)
		{
			for (s8 i = strlen(str) - 1; i >= 0; i--) file_putc(str[i]);
			for (s8 i = 0; i < padding - strlen(str); i++) file_putc('0');
		}
		else
		{
			file_write_buf((uint8_t*) str, strlen(str));
		}
		if (i != length - 1) { file_putc(','); file_putc(' '); }
	}
	file_putc('\n');
}

void usb_write_s8_array(char* name, s8* array, u8 length)
{
	file_write_buf((uint8_t*) name, strlen(name));
	file_putc(':');	file_putc(' ');
	for (u8 i = 0; i < length; i++)
	{
		itoa(array[i], str, 10);
		file_write_buf((uint8_t*) str, strlen(str));
		if (i != length - 1) { file_putc(','); file_putc(' '); }
	}
	file_putc('\n');
}

static void usb_stick_save()
{
	// file must be open for writing prior to calling this

	strcpy(str, "orca v2.4\n\n"); file_write_buf((uint8_t*) str, strlen(str));
	
	usb_write_u8_array("shared scale  1", userScalePresets[0], 16, 10, 0);
	usb_write_u8_array("shared scale  2", userScalePresets[1], 16, 10, 0);
	usb_write_u8_array("shared scale  3", userScalePresets[2], 16, 10, 0);
	usb_write_u8_array("shared scale  4", userScalePresets[3], 16, 10, 0);
	usb_write_u8_array("shared scale  5", userScalePresets[4], 16, 10, 0);
	usb_write_u8_array("shared scale  6", userScalePresets[5], 16, 10, 0);
	usb_write_u8_array("shared scale  7", userScalePresets[6], 16, 10, 0);
	usb_write_u8_array("shared scale  8", userScalePresets[7], 16, 10, 0);
	usb_write_u8_array("shared scale  9", userScalePresets[8], 16, 10, 0);
	usb_write_u8_array("shared scale 10", userScalePresets[9], 16, 10, 0);
	usb_write_u8_array("shared scale 11", userScalePresets[10], 16, 10, 0);
	usb_write_u8_array("shared scale 12", userScalePresets[11], 16, 10, 0);
	usb_write_u8_array("shared scale 13", userScalePresets[12], 16, 10, 0);
	usb_write_u8_array("shared scale 14", userScalePresets[13], 16, 10, 0);
	usb_write_u8_array("shared scale 15", userScalePresets[14], 16, 10, 0);
	usb_write_u8_array("shared scale 16", userScalePresets[15], 16, 10, 0);
	file_putc('\n');

	usb_write_u8_array("arc div sets  1", divisor_presets[0], 4, 10, 0);
	usb_write_u8_array("arc div sets  2", divisor_presets[1], 4, 10, 0);
	usb_write_u8_array("arc div sets  3", divisor_presets[2], 4, 10, 0);
	usb_write_u8_array("arc div sets  4", divisor_presets[3], 4, 10, 0);
	usb_write_u8_array("arc div sets  5", divisor_presets[4], 4, 10, 0);
	usb_write_u8_array("arc div sets  6", divisor_presets[5], 4, 10, 0);
	usb_write_u8_array("arc div sets  7", divisor_presets[6], 4, 10, 0);
	usb_write_u8_array("arc div sets  8", divisor_presets[7], 4, 10, 0);
	usb_write_u8_array("arc div sets  9", divisor_presets[8], 4, 10, 0);
	usb_write_u8_array("arc div sets 10", divisor_presets[9], 4, 10, 0);
	usb_write_u8_array("arc div sets 11", divisor_presets[10], 4, 10, 0);
	usb_write_u8_array("arc div sets 12", divisor_presets[11], 4, 10, 0);
	usb_write_u8_array("arc div sets 13", divisor_presets[12], 4, 10, 0);
	usb_write_u8_array("arc div sets 14", divisor_presets[13], 4, 10, 0);
	usb_write_u8_array("arc div sets 15", divisor_presets[14], 4, 10, 0);
	usb_write_u8_array("arc div sets 16", divisor_presets[15], 4, 10, 0);
	file_putc('\n');
	
	usb_write_u8_array("arc phase sets  1", phase_presets[0], 4, 10, 0);
	usb_write_u8_array("arc phase sets  2", phase_presets[1], 4, 10, 0);
	usb_write_u8_array("arc phase sets  3", phase_presets[2], 4, 10, 0);
	usb_write_u8_array("arc phase sets  4", phase_presets[3], 4, 10, 0);
	usb_write_u8_array("arc phase sets  5", phase_presets[4], 4, 10, 0);
	usb_write_u8_array("arc phase sets  6", phase_presets[5], 4, 10, 0);
	usb_write_u8_array("arc phase sets  7", phase_presets[6], 4, 10, 0);
	usb_write_u8_array("arc phase sets  8", phase_presets[7], 4, 10, 0);
	usb_write_u8_array("arc phase sets  9", phase_presets[8], 4, 10, 0);
	usb_write_u8_array("arc phase sets 10", phase_presets[9], 4, 10, 0);
	usb_write_u8_array("arc phase sets 11", phase_presets[10], 4, 10, 0);
	usb_write_u8_array("arc phase sets 12", phase_presets[11], 4, 10, 0);
	usb_write_u8_array("arc phase sets 13", phase_presets[12], 4, 10, 0);
	usb_write_u8_array("arc phase sets 14", phase_presets[13], 4, 10, 0);
	usb_write_u8_array("arc phase sets 15", phase_presets[14], 4, 10, 0);
	usb_write_u8_array("arc phase sets 16", phase_presets[15], 4, 10, 0);
	file_putc('\n');
	
	usb_write_u8_array("arc cv sets  1", mixer_presets[0], 2, 2, 4);
	usb_write_u8_array("arc cv sets  2", mixer_presets[1], 2, 2, 4);
	usb_write_u8_array("arc cv sets  3", mixer_presets[2], 2, 2, 4);
	usb_write_u8_array("arc cv sets  4", mixer_presets[3], 2, 2, 4);
	usb_write_u8_array("arc cv sets  5", mixer_presets[4], 2, 2, 4);
	usb_write_u8_array("arc cv sets  6", mixer_presets[5], 2, 2, 4);
	usb_write_u8_array("arc cv sets  7", mixer_presets[6], 2, 2, 4);
	usb_write_u8_array("arc cv sets  8", mixer_presets[7], 2, 2, 4);
	usb_write_u8_array("arc cv sets  9", mixer_presets[8], 2, 2, 4);
	usb_write_u8_array("arc cv sets 10", mixer_presets[9], 2, 2, 4);
	usb_write_u8_array("arc cv sets 11", mixer_presets[10], 2, 2, 4);
	usb_write_u8_array("arc cv sets 12", mixer_presets[11], 2, 2, 4);
	usb_write_u8_array("arc cv sets 13", mixer_presets[12], 2, 2, 4);
	usb_write_u8_array("arc cv sets 14", mixer_presets[13], 2, 2, 4);
	usb_write_u8_array("arc cv sets 15", mixer_presets[14], 2, 2, 4);
	usb_write_u8_array("arc cv sets 16", mixer_presets[15], 2, 2, 4);
	file_putc('\n');
	
	usb_write_param("current bank", cb + 1, 10);
	for (u8 b = 0; b < 8; b++)
	{
		usb_write_param("bank", b + 1, 10);
		usb_write_param("\tcurrent preset", banks[b].cp + 1, 10);
		for (u8 p = 0; p < 8; p++)
		{
			usb_write_param("\tpreset", p + 1, 10);
			usb_write_param("\t\tarc div set", banks[b].presets[p].divisorArc + 1, 10);
			usb_write_param("\t\tarc phase set", banks[b].presets[p].phaseArc + 1, 10);
			usb_write_param("\t\tarc cv set", banks[b].presets[p].mixerArc + 1, 10);
			usb_write_u8_array("\t\tdiv", banks[b].presets[p].divisor, 4, 10, 0);
			usb_write_u8_array("\t\tphase", banks[b].presets[p].phase, 4, 10, 0);
			usb_write_u8_array("\t\treset", banks[b].presets[p].reset, 4, 10, 0);
			usb_write_u8_array("\t\tchance", banks[b].presets[p].chance, 4, 10, 0);
			usb_write_u8_array("\t\tweight", banks[b].presets[p].weight, 4, 10, 0);
			usb_write_u8_array("\t\tgate type", banks[b].presets[p].gateType, 4, 10, 0);
			usb_write_u8_array("\t\tgate muted", banks[b].presets[p].gateMuted, 4, 10, 0);
			usb_write_u8_array("\t\tgate and/or", banks[b].presets[p].gateLogic, 4, 10, 0);
			usb_write_u8_array("\t\tgate not", banks[b].presets[p].gateNot, 4, 10, 0);
			usb_write_u8_array("\t\tgate tracks", banks[b].presets[p].gateTracks, 4, 2, 4);
			usb_write_param("\t\tselected scale A", banks[b].presets[p].scaleA + 1, 10);
			usb_write_param("\t\tselected scale B", banks[b].presets[p].scaleB + 1, 10);
			usb_write_u8_array("\t\tscale  1", banks[b].presets[p].scales[0], 16, 10, 0);
			usb_write_u8_array("\t\tscale  2", banks[b].presets[p].scales[1], 16, 10, 0);
			usb_write_u8_array("\t\tscale  3", banks[b].presets[p].scales[2], 16, 10, 0);
			usb_write_u8_array("\t\tscale  4", banks[b].presets[p].scales[3], 16, 10, 0);
			usb_write_u8_array("\t\tscale  5", banks[b].presets[p].scales[4], 16, 10, 0);
			usb_write_u8_array("\t\tscale  6", banks[b].presets[p].scales[5], 16, 10, 0);
			usb_write_u8_array("\t\tscale  7", banks[b].presets[p].scales[6], 16, 10, 0);
			usb_write_u8_array("\t\tscale  8", banks[b].presets[p].scales[7], 16, 10, 0);
			usb_write_u8_array("\t\tscale  9", banks[b].presets[p].scales[8], 16, 10, 0);
			usb_write_u8_array("\t\tscale 10", banks[b].presets[p].scales[9], 16, 10, 0);
			usb_write_u8_array("\t\tscale 11", banks[b].presets[p].scales[10], 16, 10, 0);
			usb_write_u8_array("\t\tscale 12", banks[b].presets[p].scales[11], 16, 10, 0);
			usb_write_u8_array("\t\tscale 13", banks[b].presets[p].scales[12], 16, 10, 0);
			usb_write_u8_array("\t\tscale 14", banks[b].presets[p].scales[13], 16, 10, 0);
			usb_write_u8_array("\t\tscale 15", banks[b].presets[p].scales[14], 16, 10, 0);
			usb_write_u8_array("\t\tscale 16", banks[b].presets[p].scales[15], 16, 10, 0);
			usb_write_param("\t\ttracks for CV A", banks[b].presets[p].mixerA, 2);
			usb_write_param("\t\ttracks for CV B", banks[b].presets[p].mixerB, 2);
			usb_write_param("\t\ton for CV A", banks[b].presets[p].alwaysOnA, 2);
			usb_write_param("\t\ton for CV B", banks[b].presets[p].alwaysOnB, 2);
			usb_write_s8_array("\t\trotate scale", banks[b].presets[p].rotateScale, 16);
			usb_write_s8_array("\t\trotate weights", banks[b].presets[p].rotateWeights, 16);
			usb_write_u8_array("\t\tmutate", banks[b].presets[p].mutateSeq, 8, 2, 8);
			usb_write_param("\t\tglobal reset", banks[b].presets[p].globalReset, 10);
		}
	}
	file_putc('\n');
}

u8 load_u8(char* s, u8 min, u8 max)
{
    u8 index = 0, result = 0;
    while (true)
    {
        if (s[index] == 0 || index > 254) break;
        result = result * 10 + s[index++] - '0';
    }
    if (result < min) result = min; else if (result > max) result = max;
    return result;
}

void load_u8_array(char* s, u8* array, u8 len, u8 min, u8 max)
{
    u8 index = 0, result = 0, arrayIndex = 0;
    while (true)
    {
        if (s[index] == 0 || index > 254)
        {
            if (result < min) result = min; else if (result > max) result = max;
            array[arrayIndex++] = result;
            break;
        }
        if (s[index] == ',')
        {
            if (result < min) result = min; else if (result > max) result = max;
            array[arrayIndex++] = result;
            if (arrayIndex >= len) break;
            result = 0;
        }
        else
        {
            result = result * 10 + s[index] - '0';
        }
        index++;
    }
    for (u8 i = arrayIndex; i < len; i++) array[i] = 0;
}

void load_s8_array(char* s, s8* array, u8 len, s8 min, s8 max)
{
    u8 index = 0, arrayIndex = 0, minus = 0;
    s8 result = 0;
   
    while (true)
    {
        if (s[index] == 0 || index > 254)
        {
            if (minus) result = -result;
            if (result < min) result = min; else if (result > max) result = max;
            array[arrayIndex++] = result;
            break;
        }
        if (s[index] == ',')
        {
            if (minus) result = -result;
            if (result < min) result = min; else if (result > max) result = max;
            array[arrayIndex++] = result;
            if (arrayIndex >= len) break;
            result = 0; minus = 0;
        }
        else if (s[index] == '-')
        {
            minus = 1;
        }
        else
        {
            result = result * 10 + s[index] - '0';
        }
        index++;
    }
    for (u8 i = arrayIndex; i < len; i++) array[i] = 0;
}

u8 load_u8_bin(char* s, u8 min, u8 max)
{
    u8 len = strlen(s);
    if (len == 0) return 0;
   
    u8 index = strlen(s) - 1, result = 0;
    while (true)
    {
        result = (result << 1) | (s[index] != '0');
        if (index == 0) break;
        index--;
    }
    if (result < min) result = min; else if (result > max) result = max;
    return result;
}

void load_u8_array_bin(char* s, u8* array, u8 len, u8 min, u8 max)
{
    for (u8 i = 0; i < len; i++) array[i] = 0;
    u8 slen = strlen(s);
    if (slen == 0) return;

    int index = slen - 1;
    u8 result = 0, arrayIndex = len - 1;
    while (true)
    {
        if (s[index] == ',')
        {
            if (result < min) result = min; else if (result > max) result = max;
            array[arrayIndex] = result;
            if (arrayIndex == 0) break;
            arrayIndex--;
            result = 0;
        }
        else
        {
            result = (result << 1) | (s[index] != '0');
        }
        if (index == 0)
        {
            if (result < min) result = min; else if (result > max) result = max;
            array[arrayIndex] = result;
            break;
        }
        index--;
    }
}

void process_line()
{
	if (!strcmp(str, "bank"))
	{
		bankToLoad = load_u8(svalue, 1, 8) - 1;
	}
	else if (!strcmp(str, "preset"))
	{
		presetToLoad = load_u8(svalue, 1, 8) - 1;
	}
	else if (!strcmp(str, "div"))
		load_u8_array(svalue, banks[bankToLoad].presets[presetToLoad].divisor, 4, 1, 16);
	else if (!strcmp(str, "phase"))
		load_u8_array(svalue, banks[bankToLoad].presets[presetToLoad].phase, 4, 0, 16);
	else if (!strcmp(str, "reset"))
		load_u8_array(svalue, banks[bankToLoad].presets[presetToLoad].reset, 4, 0, 16);
	else if (!strcmp(str, "chance"))
		load_u8_array(svalue, banks[bankToLoad].presets[presetToLoad].chance, 4, 0, 16);
	else if (!strcmp(str, "weight"))
		load_u8_array(svalue, banks[bankToLoad].presets[presetToLoad].weight, 4, 1, 8);
	else if (!strcmp(str, "gatetype"))
		load_u8_array(svalue, banks[bankToLoad].presets[presetToLoad].gateType, 4, 0, 2);
	else if (!strcmp(str, "gatemuted"))
		load_u8_array(svalue, banks[bankToLoad].presets[presetToLoad].gateMuted, 4, 0, 1);
	else if (!strcmp(str, "gateand/or"))
		load_u8_array(svalue, banks[bankToLoad].presets[presetToLoad].gateLogic, 4, 0, 1);
	else if (!strcmp(str, "gatenot"))
		load_u8_array(svalue, banks[bankToLoad].presets[presetToLoad].gateNot, 4, 0, 1);
	else if (!strcmp(str, "gatetracks"))
		load_u8_array_bin(svalue, banks[bankToLoad].presets[presetToLoad].gateTracks, 4, 0, 15);
	else if (!strcmp(str, "selectedscale"))
		banks[bankToLoad].presets[presetToLoad].scaleA = banks[bankToLoad].presets[presetToLoad].scaleB = load_u8(svalue, 1, 16) - 1;
	else if (!strcmp(str, "selectedscalea"))
		banks[bankToLoad].presets[presetToLoad].scaleA = load_u8(svalue, 1, 16) - 1;
	else if (!strcmp(str, "selectedscaleb"))
		banks[bankToLoad].presets[presetToLoad].scaleB = load_u8(svalue, 1, 16) - 1;
	else if (!strcmp(str, "scale1"))
		load_u8_array(svalue, banks[bankToLoad].presets[presetToLoad].scales[0], 16, 0, NOTEMAX - 1);
	else if (!strcmp(str, "scale2"))
		load_u8_array(svalue, banks[bankToLoad].presets[presetToLoad].scales[1], 16, 0, NOTEMAX - 1);
	else if (!strcmp(str, "scale3"))
		load_u8_array(svalue, banks[bankToLoad].presets[presetToLoad].scales[2], 16, 0, NOTEMAX - 1);
	else if (!strcmp(str, "scale4"))
		load_u8_array(svalue, banks[bankToLoad].presets[presetToLoad].scales[3], 16, 0, NOTEMAX - 1);
	else if (!strcmp(str, "scale5"))
		load_u8_array(svalue, banks[bankToLoad].presets[presetToLoad].scales[4], 16, 0, NOTEMAX - 1);
	else if (!strcmp(str, "scale6"))
		load_u8_array(svalue, banks[bankToLoad].presets[presetToLoad].scales[5], 16, 0, NOTEMAX - 1);
	else if (!strcmp(str, "scale7"))
		load_u8_array(svalue, banks[bankToLoad].presets[presetToLoad].scales[6], 16, 0, NOTEMAX - 1);
	else if (!strcmp(str, "scale8"))
		load_u8_array(svalue, banks[bankToLoad].presets[presetToLoad].scales[7], 16, 0, NOTEMAX - 1);
	else if (!strcmp(str, "scale9"))
		load_u8_array(svalue, banks[bankToLoad].presets[presetToLoad].scales[8], 16, 0, NOTEMAX - 1);
	else if (!strcmp(str, "scale10"))
		load_u8_array(svalue, banks[bankToLoad].presets[presetToLoad].scales[9], 16, 0, NOTEMAX - 1);
	else if (!strcmp(str, "scale11"))
		load_u8_array(svalue, banks[bankToLoad].presets[presetToLoad].scales[10], 16, 0, NOTEMAX - 1);
	else if (!strcmp(str, "scale12"))
		load_u8_array(svalue, banks[bankToLoad].presets[presetToLoad].scales[11], 16, 0, NOTEMAX - 1);
	else if (!strcmp(str, "scale13"))
		load_u8_array(svalue, banks[bankToLoad].presets[presetToLoad].scales[12], 16, 0, NOTEMAX - 1);
	else if (!strcmp(str, "scale14"))
		load_u8_array(svalue, banks[bankToLoad].presets[presetToLoad].scales[13], 16, 0, NOTEMAX - 1);
	else if (!strcmp(str, "scale15"))
		load_u8_array(svalue, banks[bankToLoad].presets[presetToLoad].scales[14], 16, 0, NOTEMAX - 1);
	else if (!strcmp(str, "scale16"))
		load_u8_array(svalue, banks[bankToLoad].presets[presetToLoad].scales[15], 16, 0, NOTEMAX - 1);
	else if (!strcmp(str, "tracksforcva"))
		banks[bankToLoad].presets[presetToLoad].mixerA = load_u8_bin(svalue, 0, 15);
	else if (!strcmp(str, "tracksforcvb"))
		banks[bankToLoad].presets[presetToLoad].mixerB = load_u8_bin(svalue, 0, 15);
	else if (!strcmp(str, "onforcva"))
		banks[bankToLoad].presets[presetToLoad].alwaysOnA = load_u8_bin(svalue, 0, 15);
	else if (!strcmp(str, "onforcvb"))
		banks[bankToLoad].presets[presetToLoad].alwaysOnB = load_u8_bin(svalue, 0, 15);
	else if (!strcmp(str, "rotatescale"))
		load_s8_array(svalue, banks[bankToLoad].presets[presetToLoad].rotateScale, 16, -2, 2);
	else if (!strcmp(str, "rotateweights"))
		load_s8_array(svalue, banks[bankToLoad].presets[presetToLoad].rotateWeights, 16, -2, 2);
	else if (!strcmp(str, "mutate"))
		load_u8_array_bin(svalue, banks[bankToLoad].presets[presetToLoad].mutateSeq, 16, 0, 255);
	else if (!strcmp(str, "globalreset"))
		banks[bankToLoad].presets[presetToLoad].globalReset = load_u8(svalue, 0, 64);
	else if (!strcmp(str, "arcdivset"))
		banks[bankToLoad].presets[presetToLoad].divisorArc = load_u8(svalue, 1, 16) - 1; 
	else if (!strcmp(str, "arcphaseset"))
		banks[bankToLoad].presets[presetToLoad].phaseArc = load_u8(svalue, 1, 16) - 1; 
	else if (!strcmp(str, "arccvset"))
		banks[bankToLoad].presets[presetToLoad].mixerArc = load_u8(svalue, 1, 16) - 1; 
	else if (!strcmp(str, "currentpreset"))
		banks[bankToLoad].cp = load_u8(svalue, 1, 8) - 1;
	else if (!strcmp(str, "currentbank"))
		cb = load_u8(svalue, 1, 8) - 1;
	else if (!strcmp(str, "sharedscale1"))
		load_u8_array(svalue, userScalePresets[0], 16, 0, NOTEMAX - 1);
	else if (!strcmp(str, "sharedscale2"))
		load_u8_array(svalue, userScalePresets[1], 16, 0, NOTEMAX - 1);
	else if (!strcmp(str, "sharedscale3"))
		load_u8_array(svalue, userScalePresets[2], 16, 0, NOTEMAX - 1);
	else if (!strcmp(str, "sharedscale4"))
		load_u8_array(svalue, userScalePresets[3], 16, 0, NOTEMAX - 1);
	else if (!strcmp(str, "sharedscale5"))
		load_u8_array(svalue, userScalePresets[4], 16, 0, NOTEMAX - 1);
	else if (!strcmp(str, "sharedscale6"))
		load_u8_array(svalue, userScalePresets[5], 16, 0, NOTEMAX - 1);
	else if (!strcmp(str, "sharedscale7"))
		load_u8_array(svalue, userScalePresets[6], 16, 0, NOTEMAX - 1);
	else if (!strcmp(str, "sharedscale8"))
		load_u8_array(svalue, userScalePresets[7], 16, 0, NOTEMAX - 1);
	else if (!strcmp(str, "sharedscale9"))
		load_u8_array(svalue, userScalePresets[8], 16, 0, NOTEMAX - 1);
	else if (!strcmp(str, "sharedscale10"))
		load_u8_array(svalue, userScalePresets[9], 16, 0, NOTEMAX - 1);
	else if (!strcmp(str, "sharedscale11"))
		load_u8_array(svalue, userScalePresets[10], 16, 0, NOTEMAX - 1);
	else if (!strcmp(str, "sharedscale12"))
		load_u8_array(svalue, userScalePresets[11], 16, 0, NOTEMAX - 1);
	else if (!strcmp(str, "sharedscale13"))
		load_u8_array(svalue, userScalePresets[12], 16, 0, NOTEMAX - 1);
	else if (!strcmp(str, "sharedscale14"))
		load_u8_array(svalue, userScalePresets[13], 16, 0, NOTEMAX - 1);
	else if (!strcmp(str, "sharedscale15"))
		load_u8_array(svalue, userScalePresets[14], 16, 0, NOTEMAX - 1);
	else if (!strcmp(str, "sharedscale16"))
		load_u8_array(svalue, userScalePresets[15], 16, 0, NOTEMAX - 1);
	else if (!strcmp(str, "arcdivsets1"))
		load_u8_array(svalue, divisor_presets[0], 4, 1, 16);
	else if (!strcmp(str, "arcdivsets2"))
		load_u8_array(svalue, divisor_presets[1], 4, 1, 16);
	else if (!strcmp(str, "arcdivsets3"))
		load_u8_array(svalue, divisor_presets[2], 4, 1, 16);
	else if (!strcmp(str, "arcdivsets4"))
		load_u8_array(svalue, divisor_presets[3], 4, 1, 16);
	else if (!strcmp(str, "arcdivsets5"))
		load_u8_array(svalue, divisor_presets[4], 4, 1, 16);
	else if (!strcmp(str, "arcdivsets6"))
		load_u8_array(svalue, divisor_presets[5], 4, 1, 16);
	else if (!strcmp(str, "arcdivsets7"))
		load_u8_array(svalue, divisor_presets[6], 4, 1, 16);
	else if (!strcmp(str, "arcdivsets8"))
		load_u8_array(svalue, divisor_presets[7], 4, 1, 16);
	else if (!strcmp(str, "arcdivsets9"))
		load_u8_array(svalue, divisor_presets[8], 4, 1, 16);
	else if (!strcmp(str, "arcdivsets10"))
		load_u8_array(svalue, divisor_presets[9], 4, 1, 16);
	else if (!strcmp(str, "arcdivsets11"))
		load_u8_array(svalue, divisor_presets[10], 4, 1, 16);
	else if (!strcmp(str, "arcdivsets12"))
		load_u8_array(svalue, divisor_presets[11], 4, 1, 16);
	else if (!strcmp(str, "arcdivsets13"))
		load_u8_array(svalue, divisor_presets[12], 4, 1, 16);
	else if (!strcmp(str, "arcdivsets14"))
		load_u8_array(svalue, divisor_presets[13], 4, 1, 16);
	else if (!strcmp(str, "arcdivsets15"))
		load_u8_array(svalue, divisor_presets[14], 4, 1, 16);
	else if (!strcmp(str, "arcdivsets16"))
		load_u8_array(svalue, divisor_presets[15], 4, 1, 16);
	else if (!strcmp(str, "arcphasesets1"))
		load_u8_array(svalue, phase_presets[0], 4, 0, 16);
	else if (!strcmp(str, "arcphasesets2"))
		load_u8_array(svalue, phase_presets[1], 4, 0, 16);
	else if (!strcmp(str, "arcphasesets3"))
		load_u8_array(svalue, phase_presets[2], 4, 0, 16);
	else if (!strcmp(str, "arcphasesets4"))
		load_u8_array(svalue, phase_presets[3], 4, 0, 16);
	else if (!strcmp(str, "arcphasesets5"))
		load_u8_array(svalue, phase_presets[4], 4, 0, 16);
	else if (!strcmp(str, "arcphasesets6"))
		load_u8_array(svalue, phase_presets[5], 4, 0, 16);
	else if (!strcmp(str, "arcphasesets7"))
		load_u8_array(svalue, phase_presets[6], 4, 0, 16);
	else if (!strcmp(str, "arcphasesets8"))
		load_u8_array(svalue, phase_presets[7], 4, 0, 16);
	else if (!strcmp(str, "arcphasesets9"))
		load_u8_array(svalue, phase_presets[8], 4, 0, 16);
	else if (!strcmp(str, "arcphasesets10"))
		load_u8_array(svalue, phase_presets[9], 4, 0, 16);
	else if (!strcmp(str, "arcphasesets11"))
		load_u8_array(svalue, phase_presets[10], 4, 0, 16);
	else if (!strcmp(str, "arcphasesets12"))
		load_u8_array(svalue, phase_presets[11], 4, 0, 16);
	else if (!strcmp(str, "arcphasesets13"))
		load_u8_array(svalue, phase_presets[12], 4, 0, 16);
	else if (!strcmp(str, "arcphasesets14"))
		load_u8_array(svalue, phase_presets[13], 4, 0, 16);
	else if (!strcmp(str, "arcphasesets15"))
		load_u8_array(svalue, phase_presets[14], 4, 0, 16);
	else if (!strcmp(str, "arcphasesets16"))
		load_u8_array(svalue, phase_presets[15], 4, 0, 16);
	else if (!strcmp(str, "arccvsets1"))
		load_u8_array_bin(svalue, mixer_presets[0], 2, 0, 15);
	else if (!strcmp(str, "arccvsets2"))
		load_u8_array_bin(svalue, mixer_presets[1], 2, 0, 15);
	else if (!strcmp(str, "arccvsets3"))
		load_u8_array_bin(svalue, mixer_presets[2], 2, 0, 15);
	else if (!strcmp(str, "arccvsets4"))
		load_u8_array_bin(svalue, mixer_presets[3], 2, 0, 15);
	else if (!strcmp(str, "arccvsets5"))
		load_u8_array_bin(svalue, mixer_presets[4], 2, 0, 15);
	else if (!strcmp(str, "arccvsets6"))
		load_u8_array_bin(svalue, mixer_presets[5], 2, 0, 15);
	else if (!strcmp(str, "arccvsets7"))
		load_u8_array_bin(svalue, mixer_presets[6], 2, 0, 15);
	else if (!strcmp(str, "arccvsets8"))
		load_u8_array_bin(svalue, mixer_presets[7], 2, 0, 15);
	else if (!strcmp(str, "arccvsets9"))
		load_u8_array_bin(svalue, mixer_presets[8], 2, 0, 15);
	else if (!strcmp(str, "arccvsets10"))
		load_u8_array_bin(svalue, mixer_presets[9], 2, 0, 15);
	else if (!strcmp(str, "arccvsets11"))
		load_u8_array_bin(svalue, mixer_presets[10], 2, 0, 15);
	else if (!strcmp(str, "arccvsets12"))
		load_u8_array_bin(svalue, mixer_presets[11], 2, 0, 15);
	else if (!strcmp(str, "arccvsets13"))
		load_u8_array_bin(svalue, mixer_presets[12], 2, 0, 15);
	else if (!strcmp(str, "arccvsets14"))
		load_u8_array_bin(svalue, mixer_presets[13], 2, 0, 15);
	else if (!strcmp(str, "arccvsets15"))
		load_u8_array_bin(svalue, mixer_presets[14], 2, 0, 15);
	else if (!strcmp(str, "arccvsets16"))
		load_u8_array_bin(svalue, mixer_presets[15], 2, 0, 15);	
}

static void usb_stick_load()
{
	// file must be open prior to calling this
	
	char c;
	u8 is_value, param_i, value_i;
	is_value = param_i = value_i = 0;
	
	while (!file_eof())
	{
		c = tolower(file_getc());
		switch (c)
		{
			case '\n':
			case '\r':
				str[param_i] = 0;
				svalue[value_i] = 0;
				process_line();
				is_value = param_i = value_i = 0;
				break;
				
			case '\t':
			case ' ':
				break;
				
			case ':':
				is_value = 1;
				break;
				
			default:
				if (is_value)
				{ 
					if (value_i < 254) svalue[value_i++] = c;
				}
				else
				{
					if (param_i < 254) str[param_i++] = c;
				}
		}
	}
	
	updatePresetCache();
}

static void usb_stick(u8 includeLoading)
{
	uint8_t usb_retry = 10;
	uint8_t lun, lun_state = 0;
	
	while (usb_retry--)
	{
		if (uhi_msc_is_available())
		{
			if (!includeLoading)
			{
				char filename[13];
				strcpy(filename, "orca_s0.txt");
				for (u8 i = 0; i < 10; i++)
				{
					filename[6] = '0' + i;
					if (nav_file_create((FS_STRING) filename)) break;
				}
				
				if (file_open(FOPEN_MODE_W)) 
				{
					usb_stick_save();
					file_close();
					nav_filelist_reset();
					nav_exit();
					usb_retry = 0;
				}
			}
		}
		else
		{
			for (lun = 0; (lun < uhi_msc_mem_get_lun()) && (lun < 8); lun++)
			{
				// Mount drive
				nav_drive_set(lun);
				if (!nav_partition_mount())
				{
					if (fs_g_status == FS_ERR_HW_NO_PRESENT)
					{
						// The test can not be done, if LUN is not present
						lun_state &= ~(1 << lun); // LUN test reseted
						continue;
					}
					lun_state |= (1 << lun); // LUN test is done.
					// ui_test_finish(false); // Test fail
					continue;
				}
				// Check if LUN has been already tested
				if (lun_state & (1 << lun))
				{
					continue;
				}

				char filename[13];

				// save
				strcpy(filename, "orca_s0.txt");
				u8 contAfterBreak = 0;
				for (u8 i = 0; i < 10; i++)
				{
					filename[6] = '0' + i;
					if (nav_file_create((FS_STRING) filename)) break;
					if (fs_g_status != FS_ERR_FILE_EXIST)
					{
						contAfterBreak = 1;
						if (fs_g_status == FS_LUN_WP)
						{
							// Test can be done only on no write protected device
							break;
						}
						lun_state |= (1 << lun); // LUN test is done.
						break;
					}
				}
				if (contAfterBreak) continue;
				
				if (!file_open(FOPEN_MODE_W))
				{
					if (fs_g_status == FS_LUN_WP)
					{
						// Test can be done only on no write protected device
						continue;
					}
					lun_state |= (1 << lun); // LUN test is done.
					continue;
				}
				
				usb_stick_save();
				file_close();
				nav_filelist_reset();
				
				lun_state |= (1 << lun); // LUN test is done.
				
				// read
				if (includeLoading)
				{
					strcpy(filename,"orca.txt");
					if (nav_filelist_findname(filename, 0))
					{
						if(file_open(FOPEN_MODE_R))
						{
							usb_stick_load();
							file_close();
						}
					}
				}
			}

			nav_exit();
			usb_retry = 0;
		}

		delay_ms(100);
	}
}

// flash commands
u8 flash_is_fresh(void) {
	return (flashy.fresh != FIRSTRUN_KEY);
}

void flash_write(void)
{
	flashc_memset8((void*)&(flashy.fresh), FIRSTRUN_KEY, 4, true);
	flashc_memcpy((void *)&flashy.userScalePresets, &userScalePresets, sizeof(userScalePresets), true);
	flashc_memcpy((void *)&flashy.divisor_presets, &divisor_presets, sizeof(divisor_presets), true);
	flashc_memcpy((void *)&flashy.phase_presets, &phase_presets, sizeof(phase_presets), true);
	flashc_memcpy((void *)&flashy.mixer_presets, &mixer_presets, sizeof(mixer_presets), true);
	flashc_memset8((void*)&(flashy.currentBank), cb, 1, true);
	flashc_memcpy((void *)&flashy.banks, &banks, sizeof(banks), true);
	timer_add(&flashSavedTimer, 140, &flashSavedTimer_callback, NULL);
	flashConfirmation = 1;
	redraw();
}

void flash_read(void)
{
	initializeValues();
	for (u8 j = 0; j < 16; j++) for (u8 k = 0; k < 16; k++) userScalePresets[j][k] = flashy.userScalePresets[j][k];
	for (u8 j = 0; j < 16; j++) for (u8 k = 0; k < 4; k++) divisor_presets[j][k] = flashy.divisor_presets[j][k];
	for (u8 j = 0; j < 16; j++) for (u8 k = 0; k < 4; k++) phase_presets[j][k] = flashy.phase_presets[j][k];
	for (u8 j = 0; j < 16; j++) for (u8 k = 0; k < 2; k++) mixer_presets[j][k] = flashy.mixer_presets[j][k];
	for (u8 b = 0; b < 8; b++) loadBank(b, 1);
    cb = flashy.currentBank;
	updatePresetCache();
}

void initializeValues(void)
{
	for (u8 j = 0; j < 16; j++) for (u8 k = 0; k < 16; k++) userScalePresets[j][k] = SCALE_PRESETS[j][k];
	for (u8 j = 0; j < 16; j++) for (u8 k = 0; k < 4; k++) divisor_presets[j][k] = DIVISOR_PRESETS[j][k];
	for (u8 j = 0; j < 16; j++) for (u8 k = 0; k < 4; k++) phase_presets[j][k] = PHASE_PRESETS[j][k];
	for (u8 j = 0; j < 16; j++) for (u8 k = 0; k < 2; k++) mixer_presets[j][k] = MIXER_PRESETS[j][k];

    cb = 0;
	u8 randDiv, randPh, randMix;
	
	for (u8 b = 0; b < 8; b++) 
	{
		banks[b].cp = 0;
		for (u8 i = 0; i < 8; i++)
		{
			banks[b].presets[i].scaleA = 0;
			banks[b].presets[i].scaleB = 0;
			for (u8 j = 0; j < 16; j++) for (u8 k = 0; k < 16; k++) banks[b].presets[i].scales[j][k] = SCALE_PRESETS[j][k];
			randDiv = random8() & 15;
			randPh = random8() & 15;
			randMix = random8() & 15;
			banks[b].presets[i].divisorArc = randDiv;
			banks[b].presets[i].phaseArc = randPh;
			banks[b].presets[i].mixerArc = randMix;
			for (u8 j = 0; j < 4; j++)
			{
				banks[b].presets[i].divisor[j] = DIVISOR_PRESETS[randDiv][j];
				banks[b].presets[i].phase[j] = PHASE_PRESETS[randPh][j];
				banks[b].presets[i].reset[j] = 0;
				banks[b].presets[i].chance[j] = 0;
				banks[b].presets[i].weight[j] = 1 << j;
				banks[b].presets[i].gateType[j] = 0;
				banks[b].presets[i].gateMuted[j] = 0;
				banks[b].presets[i].gateLogic[j] = 0;
				banks[b].presets[i].gateNot[j] = 0;
				banks[b].presets[i].gateTracks[j] = 1 << j;
			}
			banks[b].presets[i].mixerA = MIXER_PRESETS[randMix][0];
			banks[b].presets[i].mixerB = MIXER_PRESETS[randMix][1];
			banks[b].presets[i].alwaysOnA = 0;
			banks[b].presets[i].alwaysOnB = 0;
			for (u8 k = 0; k < 16; k++) banks[b].presets[i].rotateScale[k] = 0;
			for (u8 k = 0; k < 16; k++) banks[b].presets[i].rotateWeights[k] = 0;
			for (u8 k = 0; k < 8; k++) banks[b].presets[i].mutateSeq[k] = 0;
			banks[b].presets[i].globalReset = 0;
		}
	}
	
	updatePresetCache();
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
	init_i2c_slave(ORCA);

	if (flash_is_fresh())
	{
		initializeValues();
		flash_write();
	}
	else 
	{
		flash_read();
	}
	usb_stick(1);

	process_ii = &orca_process_ii;
	clock_pulse = &clock;
	clock_external = !gpio_get_pin_value(B09);

	triggerTimer_callbacks[0] = &triggerTimer0_callback;
	triggerTimer_callbacks[1] = &triggerTimer1_callback;
	triggerTimer_callbacks[2] = &triggerTimer2_callback;
	triggerTimer_callbacks[3] = &triggerTimer3_callback;
    
    triggerSettingsTimer_callbacks[0] = &triggerSettingsTimer0_callback;
    triggerSettingsTimer_callbacks[1] = &triggerSettingsTimer1_callback;
    triggerSettingsTimer_callbacks[2] = &triggerSettingsTimer2_callback;
    triggerSettingsTimer_callbacks[3] = &triggerSettingsTimer3_callback;

	timer_add(&clockTimer,120,&clockTimer_callback, NULL);
	timer_add(&keyTimer,50,&keyTimer_callback, NULL);
	timer_add(&adcTimer,100,&adcTimer_callback, NULL);
	timer_add(&cvPreviewTimer, 500, &cvPreviewTimer_callback, NULL);
	clock_temp = 10000; // out of ADC range to force tempo
    
	updateCVOutputs(0);
	updateTriggerOutputs(0);
	redraw();
	
	while (true) {
		check_events();
	}
}
