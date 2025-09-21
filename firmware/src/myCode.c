#include "myCode.h"
#include "timing.h"

#include <math.h>
#include <stdint.h>

// ===== Project-config constants kept local (table size, buffer, etc.) =====
#define PBSIZE     4096
#define SINESIZE   4096            // MUST be a power-of-two
#define PI         3.141592653589793
#define PEAK       30000           // ~ -1 dBFS headroom

// Public buffers (decls match original project)
int16_t  PlayBuff[PBSIZE];
int16_t  SineBuff[SINESIZE];
uint16_t buffer_offset = 0;

// State
enum eNoteStatus   { ready, going, finish }  noteStatus   = ready;
enum eBufferStatus { empty, finished, firstHalfReq, firstHalfDone, secondHalfReq, secondHalfDone }  bufferStatus = empty;

// Existing control variables preserved
float  noteFrequencyLeft  = 330.0f;
float  noteFrequencyRight = 330.0f;

// ======== Proper DDS: fixed-point phase accumulator ========
// Uses AUDIO_FREQUENCY_44K from Audio_Drivers.h (included via myCode.h)
#define DDS_PHASE_BITS        32u
#define DDS_INDEX_BITS        12u                   // log2(SINESIZE) (4096 -> 12)
#define DDS_FRAC_BITS         (DDS_PHASE_BITS - DDS_INDEX_BITS) // 20
#define DDS_INDEX_MASK        ((1u << DDS_INDEX_BITS) - 1u)
#define DDS_TABLE_MASK        (SINESIZE - 1u)       // requires power-of-two table

static uint32_t dds_phase = 0u;     // unsigned phase accumulator
static uint32_t dds_step  = 0u;     // phase increment per output sample

static inline void dds_set_frequency(float f_hz)
{
    // dds_step = round( f * 2^N / Fs )
    double step = ( (double)f_hz * (double)(1ull << DDS_PHASE_BITS) ) / (double)AUDIO_FREQUENCY_44K;
    if (step < 0.0) step = 0.0;
    if (step > (double)0xFFFFFFFFu) step = (double)0xFFFFFFFFu;
    dds_step = (uint32_t)llround(step);
}

// DDS table lookup with linear interpolation using index+fraction split
static inline int16_t dds_next_sample(void)
{
    // Advance phase (wrap occurs naturally in uint32_t)
    dds_phase += dds_step;

    // Split phase: [index | fraction]
    uint32_t index = (dds_phase >> DDS_FRAC_BITS) & DDS_INDEX_MASK; // 0..SINESIZE-1
    uint32_t fracQ =  dds_phase & ((1u << DDS_FRAC_BITS) - 1u);     // Q20 fractional

    // Fetch table points
    int32_t s0 = SineBuff[index & DDS_TABLE_MASK];
    int32_t s1 = SineBuff[(index + 1u) & DDS_TABLE_MASK];

    // Linear interpolation: y = s0 + (s1 - s0) * fracQ / 2^FRAC_BITS
    int32_t diff = s1 - s0;
    int32_t interp = s0 + (int32_t)(((int64_t)diff * (int64_t)fracQ) >> DDS_FRAC_BITS);
    return (int16_t)interp;
}

// ===== Board / Audio bootstrap =====
void bsp_board_init(void) {
    initAudioTimer();
    myAudioInitialisePeripherals(OUTPUT_DEVICE_AUTO, 80, AUDIO_FREQUENCY_44K);
}

void app_init_buffers_and_start(void) {
    // Build sine table
    for (int i = 0; i < SINESIZE; i++) {
        double a = (2.0 * PI * (double)i) / (double)SINESIZE;
        SineBuff[i] = (int16_t)lrint(PEAK * sin(a));
    }

    // Set initial DDS frequency from your existing control variable
    dds_set_frequency(noteFrequencyLeft);

    // --- Kick the audio path on ---
    noteStatus   = going;
    bufferStatus = firstHalfReq;

    // Prefill whole buffer so first DMA frame has valid data
    for (int i = 0; i < PBSIZE; i += 2) {
        int16_t s = dds_next_sample();
        PlayBuff[i]     = s;   // L
        PlayBuff[i + 1] = s;   // R
    }

    // Prime DMA with our buffer (driver expects samples count in BYTES)
    myAudioStartPlaying((uint16_t*)PlayBuff, PBSIZE * sizeof(int16_t));
}

void app_loop(void){
    uint32_t startFill = 0, endFill = 0;

    if (bufferStatus == firstHalfReq) {
        startFill = 0;
        endFill   = PBSIZE / 2;
        bufferStatus = firstHalfDone;
    }
    else if (bufferStatus == secondHalfReq) {
        startFill = PBSIZE / 2;
        endFill   = PBSIZE;
        bufferStatus = secondHalfDone;
    }

    if (startFill != endFill) {
        // Interleaved stereo buffer: [L, R, L, R, ...]
        for (int i = (int)startFill; i < (int)endFill; i += 2) {
            int16_t s = dds_next_sample();
            PlayBuff[i]     = s; // Left
            PlayBuff[i + 1] = s; // Right
        }
    }
}

// Debug LED (untouched from original project semantics)
uint32_t debugLED = 0;

// ===== DMA callbacks (un-gated so the ring keeps flowing) =====
void myAudioHalfTransferCallback(void) {
    bufferStatus = firstHalfReq;
}

void myAudioTransferCompleteCallback(void) {
    bufferStatus = secondHalfReq;
    myAudioChangeBuffer((int16_t *)PlayBuff, PBSIZE);
}
