#include "daisy_patch_sm.h"
#include "daisysp.h"

using namespace daisy;
using namespace patch_sm;
using namespace daisysp;

DaisyPatchSM patch;
Switch       button;

#define kBuffSize 48000 * 10 // X seconds at 48kHz
#define max_grain_count 64


struct Grain {
    size_t length = 0;
    size_t start_offset = 0;
    size_t step = 0;
};

// Loopers and the buffers they'll use
float DSY_SDRAM_BSS buffer[kBuffSize];

bool is_recording = false;
size_t record_head_pos = 0;
size_t grain_length = 48000 / 5; 
float grain_start_offset = 0.f;
float grain_spread;
float scan_speed;
unsigned int spawn_rate = 48000 / 3; // samples
uint32_t last_spawn_time = 0;
uint32_t samples_seen = 0;
size_t next_grain_to_spawn = 0;
Grain grains[max_grain_count];

float fwrap(float x, float min, float max) {
    if (max == min) return min;
    if (min > max) return fwrap(x, max, min);

    return (x >= 0 ? min : max) + fmodf(x, max - min);
}

float wrap(size_t x, size_t min, size_t max) {
    if (max == min) return min;
    if (min > max) return wrap(x, max, min);

    return (x >= 0 ? min : max) + x % (max - min);
}

float randF(float min, float max) {
    return min + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(max-min)));
}

void AudioCallback(
    AudioHandle::InputBuffer  in,
    AudioHandle::OutputBuffer out,
    size_t size
) {
    patch.ProcessAllControls();
    button.Debounce();

    spawn_rate = patch.GetAdcValue(CV_1) * 48000;
    grain_length = patch.GetAdcValue(CV_2) * 48000;
    scan_speed = (patch.GetAdcValue(CV_3) -0.5) * 4;
    grain_spread = patch.GetAdcValue(CV_4);
    
    // Toggle the record state on button press
    if(button.RisingEdge())
    {
        if (!is_recording) {
            is_recording = true;
            record_head_pos = 0;
        } else {
            is_recording = false;
        }
    }

    // Set the led to 5V if the looper is recording
    patch.SetLed(is_recording); 

    // TODO: Fade in/out the ends of the track to prevent pops
    // TODO: Windowing to get rid of pops and squeaks

    // Process audio
    for(size_t i = 0; i < size; i++)
    {
        samples_seen++;

        if (is_recording) {
            buffer[record_head_pos] = IN_L[i];
            record_head_pos++;

            if (record_head_pos >= kBuffSize) {
                is_recording = false;
            }
 
            // TODO: Get rid of this 0.5f and balance the output properly
            OUT_L[i] = IN_L[i] * 0.5f;
        } else {
            // Spawn grains
            // TODO: This will break when samples_seen wraps
            if (samples_seen - last_spawn_time >= spawn_rate) {
                last_spawn_time = samples_seen;
                grains[next_grain_to_spawn].length = grain_length;
                grains[next_grain_to_spawn].start_offset = grain_start_offset + grain_spread * randF(-0.5f, 0.5f) * record_head_pos;
                grains[next_grain_to_spawn].step = 0;

                next_grain_to_spawn = (next_grain_to_spawn + 1) % max_grain_count;
            }    

            // Calculate output
            float wet = 0.f;

            for (int j = 0; j < max_grain_count; j++) {
                if (grains[j].step <= grains[j].length) {
                    size_t buffer_index = wrap(grains[j].start_offset + grains[j].step, 0, record_head_pos);

                    // hacky bad envelope
                    // TODO: Get rid of this 0.75f and balance the output properly
                    wet += buffer[buffer_index] * std::min((grains[j].length - grains[j].step), grains[j].step) / grains[j].length * 0.75f;

                    grains[j].step++;
                }
            }

            OUT_L[i] = wet + IN_L[i] * 0.5f;
        }
    }

    grain_start_offset = fwrap(grain_start_offset + scan_speed * size, 0.f, record_head_pos);
}

int main(void)
{
    patch.Init();
    button.Init(patch.B7);
    patch.StartLog();
    patch.StartAudio(AudioCallback);

    while(1) {
        System::Delay(100);

        patch.PrintLine("grain_start_offset: " FLT_FMT3, FLT_VAR3(grain_start_offset));
    }
} 
 