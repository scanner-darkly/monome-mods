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

const u16 CHROMATIC[36] = {
	0, 34, 68, 102, 136, 170, 204, 238, 272, 306, 340, 375,
	409, 443, 477, 511, 545, 579, 613, 647, 681, 715, 750, 784,
	818, 852, 886, 920, 954, 988, 1022, 1056, 1090, 1125, 1159, 1193
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

static softTimer_t triggerTimer = { .next = NULL, .prev = NULL };
static softTimer_t confirmationTimer = { .next = NULL, .prev = NULL };
static softTimer_t cvPreviewTimer = { .next = NULL, .prev = NULL };
static softTimer_t triggerSettingsTimer = { .next = NULL, .prev = NULL };
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
// u8 debug1, debug2 = 0;

u8 arc2index = 0; // 0 - show tracks 1&2, 1 - show tracks 3&4
u8 arc4index = 0; // 0 - show tracks, 1 - show values
u8 isArc;
u8 gridParam = 0;
u64 globalCounter = 0, globalLength;
u16 counter[4] = {0, 0, 0, 0};
u8 fire[4] = {0, 0, 0, 0};
u16 prevOn[4] = {0, 0, 0, 0};
u8 flashConfirmation = 0;
u8 showTriggers = 0, showRandomized = 0, bankToShow = 8, presetToShow = 8, presetPressed = 8, bankPressed = 8;
u8 scalePressed = 0, presetModePressed = 0;
u8 prevXPressed = 16, prevXReleased = 16, prevYPressed = 8, prevYReleased = 8;
u8 randomizeX = 16, randomizeY = 4;
u8 iiTrack = 0;

struct preset {
	u8 scale;
	u8 scales[16][16];
	u8 isDivisorArc, isPhaseArc, isMixerArc;
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

u8 scale;
u8 scales[16][16];
u8 isDivisorArc, isPhaseArc, isMixerArc;
u8 divisorArc, phaseArc, mixerArc;
u8 divisor[4], phase[4], reset[4], chance[4], weight[4];
u8 gateType[4], gateMuted[4], gateLogic[4], gateNot[4], gateTracks[4];
u8 mixerA, mixerB, alwaysOnA, alwaysOnB;
s8 rotateScale[16], rotateWeights[16]; 
u8 mutateSeq[8];
u8 globalReset;

typedef const struct {
	u8 fresh;
    u8 currentBank;
	struct bank banks[8];
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
static void orca_process_ii(uint8_t i, int d);

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
u8 isTrackDead(u8 divisor, u8 phase, u8 reset, u8 globalReset);

void redraw(void);
void redrawArc(void);
void redrawGrid(void);
void updateOutputs(void);
void showValue(u8 value);
static void triggerTimer_callback(void* o);
static void confirmationTimer_callback(void* o);
static void cvPreviewTimer_callback(void* o);
static void triggerSettingsTimer_callback(void* o);
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
	monomeLedBuffer[2] = rotateScale[seqOffset] != 0 && showTriggers ? 10 : 5;
	monomeLedBuffer[18] = rotateWeights[seqOffset] != 0 && showTriggers ? 10 : 5;
	monomeLedBuffer[34] = getMutateSeq(globalCounter & 63) != 0 && showTriggers ? 10 : 5;
	monomeLedBuffer[50] = globalReset && !globalCounter && showTriggers ? 10 : 5;

    if (gridParam == COUNTERS)
    {
		for (u8 i = 0; i < 4; i++)
			for (u8 led = 0; led < 16; led++)
				monomeLedBuffer[64+(i<<4)+led] = led < (counter[i] & 15) ? 15 : 0;
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
				monomeLedBuffer[64 + i] = monomeLedBuffer[96 + i] = 0;
				monomeLedBuffer[80 + i] = i;
				monomeLedBuffer[112 + i] = i == scale ? 15 : 8;
			}
        }
        else
        {
            u8 noteIndex;
            u8 cvA = 0;
            
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
                noteIndex = scales[scale][(currentScaleRow << 2) + x];
                monomeLedBuffer[68 + (x << 4) + (noteIndex % 12)] = 4 + ((noteIndex / 12) << 2);
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
                    if (currentOn && ((alwaysOnA | mixerA) & (1 << seq))) cvA += weight[seq];
                }
                cvA &= 0xf;
                
                // highlight it in the 4x4 block
                monomeLedBuffer[64+((cvA>>2)<<4)+(cvA&3)] = 15;

                // highlight it in the note space
                if ((cvA>>2) == currentScaleRow)
                {
                    noteIndex = scales[scale][cvA] % 12;
                    monomeLedBuffer[68 + ((cvA & 3) << 4) + noteIndex] += 3;
                }
            }
        }    
	}
	else if (gridParam == SETTINGS)
	{
		monomeLedBuffer[17] = 15;
		
		u8 addFire, add[4];
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
			addFire = fire[i] && (gateType[i] == 2 || showTriggers) ? 3 : 0;
			
			monomeLedBuffer[seqOffset] = (gateMuted[i] ? 0 : 12) + addFire;
			monomeLedBuffer[seqOffset + 1] = gateType[i] * 6 + addFire;
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
			for (u8 i = 0; i < 4; i++)
			{
				for (u8 led = 0; led < 16; led++)
				{
					monomeLedBuffer[64+(i<<4)+led] = showRandomized && (i == randomizeY) ? 0 : ((i << 1) + ((led * 10) >> 4));
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
            monomeLedBuffer[81+(i<<1)] = i == cb ? 13 : (i & 1 ? 5 : 0);
        }
        for (u8 i = 0; i < 8; i++)
        {
            monomeLedBuffer[96+(i<<1)] = i == banks[cb].cp ? 13 : (i & 1 ? 0 : 5);
            monomeLedBuffer[97+(i<<1)] = i == banks[cb].cp ? 13 : (i & 1 ? 0 : 5);
            monomeLedBuffer[112+(i<<1)] = i == banks[cb].cp ? 13 : (i & 1 ? 0 : 5);
            monomeLedBuffer[113+(i<<1)] = i == banks[cb].cp ? 13 : (i & 1 ? 0 : 5);
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
			monomeLedBuffer[64 + i] = (i < globalReset ? 12 : 2) + add;
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
			monomeLedBuffer[13 - led + seqOffset] = currentOn ? 15 - led : 0;
			
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
		
		monomeLedBuffer[14 + seqOffset] = alwaysOnA & (1 << seq) ? 15 : (mixerA & (1 << seq) ? (currentOn ? 12 : 6) : 0);
		monomeLedBuffer[15 + seqOffset] = alwaysOnB & (1 << seq) ? 15 : (mixerB & (1 << seq) ? (currentOn ? 12 : 6) : 0);
	}
	
	if (valueToShow == 5) // show scale
	{
		for (u8 led = 0; led < 16; led++)
			monomeLedBuffer[led+112] = led == scale ? 15 : 6;
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
	
	/*
	for (u8 led = 0; led < 16; led++)
		monomeLedBuffer[64+led] = led < presetPressed ? 15 : 0;
	for (u8 led = 0; led < 16; led++)
		monomeLedBuffer[80+led] = led < debug2 ? 15 : 0;
	*/
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
		if (valueToShow == 1 || arc4index) // divisorArc 0-15
		{
			level = isDivisorArc ? 15 : 0;
			for (u8 led = 0; led < 64; led++)
				monomeLedBuffer[led] = !(led & 3) ? 5 : ((led < ((divisorArc + 1) << 2)) && (led >= (divisorArc << 2)) ? level : 0);
		}
		if (valueToShow == 2 || arc4index) // phaseArc 0-15
		{
			level = isPhaseArc ? 15 : 0;
			for (u8 led = 0; led < 64; led++)
				monomeLedBuffer[led + 64] = !(led & 3) ? 5 : ((led < ((phaseArc + 1) << 2)) && (led >= (phaseArc << 2)) ? level : 0);
		}
		if (valueToShow == 3 || arc4index) // mixerArc 0-15
		{
			level = isMixerArc ? 15 : 0;
			for (u8 led = 0; led < 64; led++)
				monomeLedBuffer[led + 128] = !(led & 3) ? 5 : ((led < ((mixerArc + 1) << 2)) && (led >= (mixerArc << 2)) ? level : 0);
		}
		if (valueToShow == 4 || arc4index) // preset 0-7
		{
			u8 pres = (cb << 3) + banks[cb].cp;
			for (u8 led = 0; led < 64; led++)
				monomeLedBuffer[led + 192] = led == pres ? 15 : (led & 7 ? 1 : 6);
		}
		if (valueToShow == 5) // scale 0-15
		{
			for (u8 led = 0; led < 64; led++)
				monomeLedBuffer[led] = !(led & 3) ? 5 : ((led < ((scale + 1) << 2)) && (led >= (scale << 2)) ? 15 : 0);
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

void updateOutputs()
{
	timer_remove(&triggerTimer);

	if (gridParam == SCALE && isScalePreview)
	{
		if (gateMuted[0]) gpio_clr_gpio_pin(TRIGGERS[0]); else gpio_set_gpio_pin(TRIGGERS[0]);
		if (gateMuted[1]) gpio_clr_gpio_pin(TRIGGERS[1]); else gpio_set_gpio_pin(TRIGGERS[1]);
		if (gateMuted[2]) gpio_clr_gpio_pin(TRIGGERS[2]); else gpio_set_gpio_pin(TRIGGERS[2]);
		if (gateMuted[3]) gpio_clr_gpio_pin(TRIGGERS[3]); else gpio_set_gpio_pin(TRIGGERS[3]);
		cv0 = cv1 = CHROMATIC[scales[scale][(currentScaleRow<<2)+currentScaleColumn]];
	}
	else
	{
		u8 cvA = 0, cvB = 0;
		u16 currentOn, trigOn, offset;
		u8 trackIncluded;
		fire[0] = gateLogic[0] && gateTracks[0];
		fire[1] = gateLogic[1] && gateTracks[1];
		fire[2] = gateLogic[2] && gateTracks[2];
		fire[3] = gateLogic[3] && gateTracks[3];
		
		for (u8 seq = 0; seq < 4; seq++)
		{
			offset = counter[seq] + (divisor[seq] << 6) - phase[seq];
			currentOn = (offset / divisor[seq]) & 1;
			
			if (chance[seq] < ((rnd() % 20)+1))
			{
				if ((alwaysOnA & (1 << seq)) | (currentOn && (mixerA & (1 << seq)))) cvA += weight[seq];
				if ((alwaysOnB & (1 << seq)) | (currentOn && (mixerB & (1 << seq)))) cvB += weight[seq]; 
				
				for (u8 trig = 0; trig < 4; trig++)
				{
					trackIncluded = gateTracks[trig] & (1 << seq);
					trigOn = !gateType[trig] ? prevOn[seq] != currentOn : !prevOn[seq] && currentOn;
					
					if (gateLogic[trig]) // AND
					{
						if (trackIncluded) fire[trig] &= gateType[trig] == 2 ? currentOn : trigOn;
					}
					else // OR
					{
						if (trackIncluded) fire[trig] |= gateType[trig] == 2 ? currentOn : trigOn;
					}
				}
			}
			
			prevOn[seq] = currentOn;
		}

		if (gateNot[0]) fire[0] = !fire[0];
		if (gateNot[1]) fire[1] = !fire[1];
		if (gateNot[2]) fire[2] = !fire[2];
		if (gateNot[3]) fire[3] = !fire[3];

		for (u8 trig = 0; trig < 4; trig++)
		{
			if (gateMuted[trig])
				gpio_clr_gpio_pin(TRIGGERS[trig]);
			else if (fire[trig])
				gpio_set_gpio_pin(TRIGGERS[trig]);
			else
				gpio_clr_gpio_pin(TRIGGERS[trig]);
		}
		
		cv0 = CHROMATIC[scales[scale][cvA & 0xf]];
		cv1 = CHROMATIC[scales[scale][cvB & 0xf]];
	}

	timer_add(&triggerTimer, 10, &triggerTimer_callback, NULL);
	
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
		for (u8 j = 0; j < 16; j++) banks[cb].presets[banks[cb].cp].scales[banks[cb].presets[banks[cb].cp].scale][j] = random8() % 36;
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
	banks[cb].presets[banks[cb].cp].alwaysOnA = random8() & 15;
	banks[cb].presets[banks[cb].cp].alwaysOnB = random8() & 15;
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
	
	if (paramCount > 2)
	{
		for (u8 i = 0; i < 4; i++)
			banks[cb].presets[banks[cb].cp].weight[i] = weight[i] = (random8() & 7) + 1;
	}

	if (paramCount > 3)
		rotateScales(random8() % max);
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
			u8 lastScale = scales[scale][15];
			scales[scale][15] = scales[scale][14];
			scales[scale][14] = scales[scale][13];
			scales[scale][13] = scales[scale][12];
			scales[scale][12] = scales[scale][11];
			scales[scale][11] = scales[scale][10];
			scales[scale][10] = scales[scale][9];
			scales[scale][9] = scales[scale][8];
			scales[scale][8] = scales[scale][7];
			scales[scale][7] = scales[scale][6];
			scales[scale][6] = scales[scale][5];
			scales[scale][5] = scales[scale][4];
			scales[scale][4] = scales[scale][3];
			scales[scale][3] = scales[scale][2];
			scales[scale][2] = scales[scale][1];
			scales[scale][1] = scales[scale][0];
			scales[scale][0] = lastScale;
		}
		for (u8 i = 0; i < 8; i++) for (u8 j = 0; j < 8; j++)
			banks[cb].presets[banks[cb].cp].scales[i][j] = scales[i][j];
	} 
	else
	{
		for (s8 j = 0; j < amount; j++)
		{
			u8 firstScale = scales[scale][0];
			scales[scale][0] = scales[scale][1];
			scales[scale][1] = scales[scale][2];
			scales[scale][2] = scales[scale][3];
			scales[scale][3] = scales[scale][4];
			scales[scale][4] = scales[scale][5];
			scales[scale][5] = scales[scale][6];
			scales[scale][6] = scales[scale][7];
			scales[scale][7] = scales[scale][8];
			scales[scale][8] = scales[scale][9];
			scales[scale][9] = scales[scale][10];
			scales[scale][10] = scales[scale][11];
			scales[scale][11] = scales[scale][12];
			scales[scale][12] = scales[scale][13];
			scales[scale][13] = scales[scale][14];
			scales[scale][14] = scales[scale][15];
			scales[scale][15] = firstScale;
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
		
		updateOutputs();
		showTriggers = 1;
		timer_add(&triggerSettingsTimer, 100, &triggerSettingsTimer_callback, NULL);
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

static void triggerTimer_callback(void* o) {
	timer_remove(&triggerTimer);
	for (u8 trig = 0; trig < 4; trig++)
	{
		if (gateType[trig] != 2) gpio_clr_gpio_pin(TRIGGERS[trig]);
	}
}

static void triggerSettingsTimer_callback(void* o) {
	timer_remove(&triggerSettingsTimer);
	showTriggers = 0;
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
	if (monome_device() == eDeviceGrid)
	{
		isDivisorArc = isPhaseArc = isMixerArc = 0;
		banks[cb].presets[banks[cb].cp].isDivisorArc = 0;
		banks[cb].presets[banks[cb].cp].isPhaseArc = 0;
		banks[cb].presets[banks[cb].cp].isMixerArc = 0;
	}
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
			if (monome_encs() == 2)
				arc2index = !arc2index;
			else
				arc4index = !arc4index;
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
	u8 newPotValue = (adc[1] >> 8) & 15; // should be in the range 0-15
	if (prevPotValue == 16) // haven't read the value yet
	{
		prevPotValue = newPotValue;
	}
	else if (newPotValue != prevPotValue)
	{
		prevPotValue = scale = banks[cb].presets[banks[cb].cp].scale = newPotValue;
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


static void orca_process_ii(uint8_t i, int d)
{
	switch(i)
	{
		case ORCA_TRACK:
			iiTrack = abs(d) & 3;
			break;
		
		case ORCA_DIVISOR:
			divisor[iiTrack] = banks[cb].presets[banks[cb].cp].divisor[iiTrack] = (abs(d - 1) & 15) + 1;
			adjustCounter(iiTrack);
			break;
		
		case ORCA_PHASE:
			phase[iiTrack] = banks[cb].presets[banks[cb].cp].phase[iiTrack] = abs(d) % 17;
			break;
		
		case ORCA_WEIGHT:
			weight[iiTrack] = banks[cb].presets[banks[cb].cp].weight[iiTrack] = (abs(d - 1) & 7) + 1;
			break;
		
		case ORCA_MUTE:
			gateMuted[iiTrack] = banks[cb].presets[banks[cb].cp].gateMuted[iiTrack] = d ? !0 : 0;
			break;
		
		case ORCA_SCALE:
			scale = banks[cb].presets[banks[cb].cp].scale = abs(d) & 15;
			break;
		
		case ORCA_BANK:
			cb = abs(d) & 7;
			bankToShow = cb;
			updatePresetCache();
			break;
		
		case ORCA_PRESET:
			banks[cb].cp = abs(d) & 7;
			presetToShow = banks[cb].cp;
			updatePresetCache();
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
			break;
		
		case ORCA_ROTATES:
			rotateScales(d % 16);
			break;
		
		case ORCA_ROTATEW:
			rotateWeights_(d % 4);
			break;
		
		case ORCA_CVA:
			if (d >= 0)
			{
				mixerA = banks[cb].presets[banks[cb].cp].mixerA = d & 15;
			}
			else
			{
				d = -d;
				mixerA = alwaysOnA = 0;
				if ((d / 1000) % 10 == 1) mixerA = 0b1000; else if ((d / 1000) % 10 == 2) alwaysOnA = 0b1000;
				if ((d / 100 ) % 10 == 1) mixerA |= 0b0100; else if ((d / 100) % 10 == 2) alwaysOnA |= 0b0100;
				if ((d / 10 ) % 10 == 1) mixerA |= 0b0010; else if ((d / 10) % 10 == 2) alwaysOnA |= 0b0010;
				if (d % 10 == 1) mixerA |= 0b0001; else if (d % 10 == 2) alwaysOnA |= 0b0001;
				banks[cb].presets[banks[cb].cp].mixerA = mixerA;
				banks[cb].presets[banks[cb].cp].alwaysOnA = alwaysOnA;
			}
			break;
		
		case ORCA_CVB:
			if (d >= 0)
			{
				mixerB = banks[cb].presets[banks[cb].cp].mixerB = d & 15;
			}
			else
			{
				d = -d;
				mixerB = alwaysOnB = 0;
				if ((d / 1000) % 10 == 1) mixerB = 0b1000; else if ((d / 1000) % 10 == 2) alwaysOnB = 0b1000;
				if ((d / 100 ) % 10 == 1) mixerB |= 0b0100; else if ((d / 100) % 10 == 2) alwaysOnB |= 0b0100;
				if ((d / 10 ) % 10 == 1) mixerB |= 0b0010; else if ((d / 10) % 10 == 2) alwaysOnB |= 0b0010;
				if (d % 10 == 1) mixerB |= 0b0001; else if (d % 10 == 2) alwaysOnB |= 0b0001;
				banks[cb].presets[banks[cb].cp].mixerB = mixerB;
				banks[cb].presets[banks[cb].cp].alwaysOnB = alwaysOnB;
			}
			break;
		
		case ORCA_GRESET:
			break;

		case ORCA_CLOCK:
			break;
		
		case ORCA_RESET:
			break;
		
	}
	
	if (i != ORCA_TRACK)
	{
		updateOutputs();
		redraw();
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
	
	if (monome_encs() == 2) // arc2
	{
		switch(n)
		{
			case 0:
				if (delta < 0)
				{
					if (!isDivisorArc)
					{
						banks[cb].presets[banks[cb].cp].isDivisorArc = isDivisorArc = 1;
						divisorArc = 0;
					}
					else if (++divisorArc > 15) divisorArc = 0;
					banks[cb].presets[banks[cb].cp].divisorArc = divisorArc;
					for (u8 seq = 0; seq < 4; seq++)
					{
						banks[cb].presets[banks[cb].cp].divisor[seq] = divisor[seq] = DIVISOR_PRESETS[divisorArc][seq];
					}						
					adjustAllCounters();
					showValue(1);
				} 
				else 
				{ 
					if (!isPhaseArc)
					{
						banks[cb].presets[banks[cb].cp].isPhaseArc = isPhaseArc = 1;
						phaseArc = 0;
					}
					else if (++phaseArc > 15) phaseArc = 0;						
					banks[cb].presets[banks[cb].cp].phaseArc = phaseArc;
					for (u8 seq = 0; seq < 4; seq++) banks[cb].presets[banks[cb].cp].phase[seq] = phase[seq] = PHASE_PRESETS[phaseArc][seq];
					showValue(2);
				}
				break;
			case 1:
				if (delta < 0)
				{
					if (!isMixerArc)
					{
						banks[cb].presets[banks[cb].cp].isMixerArc = isMixerArc = 1;
						mixerArc = 0;
					} else if (++mixerArc > 15) mixerArc = 0;
					banks[cb].presets[banks[cb].cp].mixerArc = mixerArc;
					banks[cb].presets[banks[cb].cp].mixerA = mixerA = MIXER_PRESETS[mixerArc][0];
					banks[cb].presets[banks[cb].cp].mixerB = mixerB = MIXER_PRESETS[mixerArc][1];
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
					banks[cb].presets[banks[cb].cp].isDivisorArc = isDivisorArc = 1;
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
				for (u8 seq = 0; seq < 4; seq++) banks[cb].presets[banks[cb].cp].divisor[seq] = divisor[seq] = DIVISOR_PRESETS[divisorArc][seq];
				adjustAllCounters();
				showValue(1);
				break;
			case 1:
				if (!isPhaseArc)
				{
					banks[cb].presets[banks[cb].cp].isPhaseArc = isPhaseArc = 1;
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
				for (u8 seq = 0; seq < 4; seq++) banks[cb].presets[banks[cb].cp].phase[seq] = phase[seq] = PHASE_PRESETS[phaseArc][seq];
				showValue(2);
				break;
			case 2:
				if (!isMixerArc)
				{
					banks[cb].presets[banks[cb].cp].isMixerArc = isMixerArc = 1;
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
				banks[cb].presets[banks[cb].cp].mixerA = mixerA = MIXER_PRESETS[mixerArc][0];
				banks[cb].presets[banks[cb].cp].mixerB = mixerB = MIXER_PRESETS[mixerArc][1];
				showValue(3);
				break;
			case 3:
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

	if (!z) 
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
    
		if (gridParam == PRESETS)
		{
			if (y < 6 && bankPressed == (x >> 1)) bankPressed = 8;
			if (y > 5 && presetPressed == (x >> 1)) presetPressed = 8;
		}
	
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
			if (doublePress) globalCounter = globalLength;
		}
		
		redraw();
		return;
	}
	
	if (x == 14 && y < 4) // CV A mixing
	{
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
		redraw();
		return;
	}

	if (x == 15 && y < 4) // CV B mixing
	{
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
			if (y == 5)
			{
				for (u8 i = 0; i < 16; i++)
				{
					banks[cb].presets[banks[cb].cp].scales[scale][i] = scales[scale][i] = SCALE_PRESETS[x][i];
				}
			}
			else if (y == 7)
			{
				banks[cb].presets[banks[cb].cp].scale = scale = x;
			}
			redraw();
			return;
		}
		
		if (x < 4)
		{
			currentScaleRow = y - 4;
			currentScaleColumn = x;
            prevSelectedScaleColumn = 4;
            
			if (isScalePreview) updateOutputs();
			redraw();
			return;
		}
        
        x -= 4;
        currentScaleColumn = y - 4; // yep, y sets column here as the "piano roll" is pivoted
        u8 noteIndex = scales[scale][(currentScaleRow<<2)+currentScaleColumn]; // currently selected note
        
		if (noteIndex % 12 == x) // same note
		{
            if (prevSelectedScaleColumn == currentScaleColumn)
                noteIndex = (noteIndex + 12) % 36;
		}
		else
		{
			noteIndex = x + noteIndex / 12 * 12;
		}
        prevSelectedScaleColumn = currentScaleColumn;
        
		banks[cb].presets[banks[cb].cp].scales[scale][(currentScaleRow<<2)+currentScaleColumn] = scales[scale][(currentScaleRow<<2)+currentScaleColumn] = noteIndex;
		if (isScalePreview) updateOutputs();
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
	banks[b].presets[p].scale = flashy.banks[b].presets[p].scale;
	for (u8 j = 0; j < 16; j++) for (u8 k = 0; k < 16; k++) banks[b].presets[p].scales[j][k] = flashy.banks[b].presets[p].scales[j][k];
	banks[b].presets[p].isDivisorArc = flashy.banks[b].presets[p].isDivisorArc;
	banks[b].presets[p].isPhaseArc = flashy.banks[b].presets[p].isPhaseArc;
	banks[b].presets[p].isMixerArc = flashy.banks[b].presets[p].isMixerArc;
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
	banks[cb].presets[dest].scale = banks[cb].presets[source].scale;
	for (u8 j = 0; j < 16; j++) for (u8 k = 0; k < 16; k++) banks[cb].presets[dest].scales[j][k] = banks[cb].presets[source].scales[j][k];
	banks[cb].presets[dest].isDivisorArc = banks[cb].presets[source].isDivisorArc;
	banks[cb].presets[dest].isPhaseArc = banks[cb].presets[source].isPhaseArc;
	banks[cb].presets[dest].isMixerArc = banks[cb].presets[source].isMixerArc;
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
		banks[dest].presets[i].scale = banks[source].presets[i].scale;
		for (u8 j = 0; j < 16; j++) for (u8 k = 0; k < 16; k++) banks[dest].presets[i].scales[j][k] = banks[source].presets[i].scales[j][k];
		banks[dest].presets[i].isDivisorArc = banks[source].presets[i].isDivisorArc;
		banks[dest].presets[i].isPhaseArc = banks[source].presets[i].isPhaseArc;
		banks[dest].presets[i].isMixerArc = banks[source].presets[i].isMixerArc;
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
	scale = banks[cb].presets[banks[cb].cp].scale;
	for (u8 j = 0; j < 16; j++) for (u8 k = 0; k < 16; k++) scales[j][k] = banks[cb].presets[banks[cb].cp].scales[j][k];
	isDivisorArc = banks[cb].presets[banks[cb].cp].isDivisorArc;
	isPhaseArc = banks[cb].presets[banks[cb].cp].isPhaseArc;
	isMixerArc = banks[cb].presets[banks[cb].cp].isMixerArc;
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

// flash commands
u8 flash_is_fresh(void) {
	return (flashy.fresh != FIRSTRUN_KEY);
}

void flash_write(void)
{
	flashc_memset8((void*)&(flashy.fresh), FIRSTRUN_KEY, 4, true);
	flashc_memset8((void*)&(flashy.currentBank), cb, 1, true);
	flashc_memcpy((void *)&flashy.banks, &banks, sizeof(banks), true);
	timer_add(&flashSavedTimer, 140, &flashSavedTimer_callback, NULL);
	flashConfirmation = 1;
	redraw();
}

void flash_read(void) {
	initializeValues();

	for (u8 b = 0; b < 8; b++) 
		loadBank(b, 1);

    cb = flashy.currentBank;
	updatePresetCache();
}

void initializeValues(void)
{
    cb = 0;
	u8 randDiv, randPh, randMix;
	
	for (u8 b = 0; b < 8; b++) 
	{
		banks[b].cp = 0;
		for (u8 i = 0; i < 8; i++)
		{
			banks[b].presets[i].scale = 0;
			for (u8 j = 0; j < 16; j++) for (u8 k = 0; k < 16; k++) banks[b].presets[i].scales[j][k] = SCALE_PRESETS[j][k];
			banks[b].presets[i].isDivisorArc = 1;
			banks[b].presets[i].isPhaseArc = 1;
			banks[b].presets[i].isMixerArc = 1;
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
		flash_read();

	process_ii = &orca_process_ii;
	clock_pulse = &clock;
	clock_external = !gpio_get_pin_value(B09);

	timer_add(&clockTimer,120,&clockTimer_callback, NULL);
	timer_add(&keyTimer,50,&keyTimer_callback, NULL);
	timer_add(&adcTimer,100,&adcTimer_callback, NULL);
	timer_add(&cvPreviewTimer, 500, &cvPreviewTimer_callback, NULL);
	clock_temp = 10000; // out of ADC range to force tempo
    
	updateOutputs();
	redraw();
	
	while (true) {
		check_events();
	}
}
