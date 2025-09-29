#include "myCode.h"
#include "timing.h"
#include "uart.h"


#include <math.h>
#include <stdint.h>

// ===== Project-config constants kept local (table size, buffer, etc.) =====
#define PBSIZE     4096
#define SINESIZE   4096           // MUST be a power-of-two
#define PI         3.141592653589793238462643383279502884197169
#define PEAK       30000           // ~ -1 dBFS headroom

// Public buffers (decls match original project)
int32_t  PlayBuff[PBSIZE];
int32_t  SineBuff[SINESIZE];
uint32_t buffer_offset = 0;

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


// FOR FUN ---------------------------------
// ---- Simple harmonic control (no glide, phase-continuous) ----
static volatile uint8_t  freq_update_pending = 0;
static volatile uint32_t pending_step = 0;

// Use your existing base (you set noteFrequencyLeft = 330.0f above)
static float f0_hz = 330.0f;

// Compute the DDS step for a target frequency without touching phase
static inline uint32_t step_for_freq(float f_hz)
{
    if (f_hz < 0.f) f_hz = 0.f;
    const float nyq = AUDIO_FREQUENCY_96K * 0.5f;
    if (f_hz > nyq) f_hz = nyq;

    // same math as dds_set_frequency, but returns the step
    double step = ((double)f_hz * (double)(1ull << DDS_PHASE_BITS)) / (double)AUDIO_FREQUENCY_96K;
    if (step < 0.0) step = 0.0;
    if (step > (double)0xFFFFFFFFu) step = (double)0xFFFFFFFFu;
    return (uint32_t)llround(step);
}

// Eat all pending UART chars, remember the last digit 1..9,
// and prepare the new DDS step (to apply at the next buffer start)
static inline void poll_keys_and_prepare_freq(void)
{
    int last = -1;
    for (;;) {
        int c = uart2_getchar_nonblocking();
        if (c < 0) break;
        if (c >= '1' && c <= '9') last = c;
    }
    if (last >= 0) {
        int n = last - '0';              // 1..9
        float new_f = f0_hz * (float)n;  // nth harmonic
        pending_step = step_for_freq(new_f);
        freq_update_pending = 1;
    }
}

// Apply pending frequency change (phase stays continuous)
static inline void apply_pending_freq_if_any(void)
{
    if (freq_update_pending) {
        dds_step = pending_step;         // instant jump, no phase reset
        freq_update_pending = 0;
    }
}
// ---------------------------------





static inline void dds_set_frequency(float f_hz)
{
    // dds_step = round( f * 2^N / Fs )
    double step = ( (double)f_hz * (double)(1ull << DDS_PHASE_BITS) ) / (double)AUDIO_FREQUENCY_96K;
    if (step < 0.0) step = 0.0;
    if (step > (double)0xFFFFFFFFu) step = (double)0xFFFFFFFFu;
    dds_step = (uint32_t)llround(step);
}

// DDS table lookup with linear interpolation using index+fraction split
static inline int32_t dds_next_sample(void)
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
    myAudioInitialisePeripherals(OUTPUT_DEVICE_AUTO, 80, AUDIO_FREQUENCY_96K);
}

void app_init_buffers_and_start(void) {
    // Build sine table
    for (int i = 0; i < SINESIZE; i++) {
        double a = (2.0 * PI * (double)i) / (double)SINESIZE;
        SineBuff[i] = (int32_t)lrint(PEAK * sin(a));
    }

    // Set initial DDS frequency from your existing control variable
    dds_set_frequency(noteFrequencyLeft);

    // --- Kick the audio path on ---
    noteStatus   = going;
    bufferStatus = firstHalfReq;

    // Prefill whole buffer so first DMA frame has valid data
    for (int i = 0; i < PBSIZE; i += 2) {
        int32_t s = dds_next_sample();
        PlayBuff[i]     = s;   // L
        PlayBuff[i + 1] = s;   // R
    }

    // Prime DMA with our buffer (driver expects samples count in BYTES)
    myAudioStartPlaying((int32_t*)PlayBuff, PBSIZE * sizeof(int32_t));
}

void app_loop(void){
    uint32_t startFill = 0, endFill = 0;

    if (bufferStatus == firstHalfReq) {
        startFill = 0;
        endFill   = PBSIZE / 2;
        bufferStatus = firstHalfDone;

        // NEW: fast control poll + apply just before filling
		poll_keys_and_prepare_freq();
		apply_pending_freq_if_any();
    }
    else if (bufferStatus == secondHalfReq) {
        startFill = PBSIZE / 2;
        endFill   = PBSIZE;
        bufferStatus = secondHalfDone;

        // NEW: fast control poll + apply just before filling
		poll_keys_and_prepare_freq();
		apply_pending_freq_if_any();
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
    myAudioChangeBuffer((int32_t *)PlayBuff, PBSIZE);
}
