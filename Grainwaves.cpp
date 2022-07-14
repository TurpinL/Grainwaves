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
unsigned int spawn_rate = 1000 / 3; // ms
uint32_t last_spawn_time = 0;
size_t next_grain_to_spawn = 0;
Grain grains[max_grain_count];

void AudioCallback(
    AudioHandle::InputBuffer  in,
    AudioHandle::OutputBuffer out,
    size_t size
) {
    patch.ProcessAllControls();
    button.Debounce();

    spawn_rate = 1000 * patch.GetAdcValue(CV_1);
    grain_length = patch.GetAdcValue(CV_2) * 48000;
    
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

    if (!is_recording) {
        if (System::GetNow() - last_spawn_time >= spawn_rate) {
            last_spawn_time = System::GetNow();
            grains[next_grain_to_spawn].length = grain_length;
            grains[next_grain_to_spawn].start_offset = grain_start_offset;
            grains[next_grain_to_spawn].step = 0;

            next_grain_to_spawn = (next_grain_to_spawn + 1);
            if (next_grain_to_spawn >= max_grain_count) {
                next_grain_to_spawn = 0;
            }
        }
    }

    // TODO: Fade in/out the ends of the track to prevent pops
    // TODO: Windowing to get rid of pops and squeaks

    // Process audio
    for(size_t i = 0; i < size; i++)
    {
        if (is_recording) {
            buffer[record_head_pos] = IN_L[i];
            record_head_pos++;

            if (record_head_pos >= kBuffSize) {
                is_recording = false;
            }
 
            // TODO: Get rid of this 0.5f and balance the output properly
            OUT_L[i] = IN_L[i] * 0.5f;
        } else {
            float wet = 0.f;

            for (int j = 0; j < max_grain_count; j++) {
                if (grains[j].step <= grains[j].length) {
                    size_t buffer_index = (grains[j].start_offset + grains[j].step);
                    if (buffer_index > record_head_pos) {
                        buffer_index -= record_head_pos;
                    }

                    // hacky bad envelope
                    // TODO: Get rid of this 0.75f and balance the output properly
                    wet += buffer[buffer_index] * std::min((grains[j].length - grains[j].step), grains[j].step) / grains[j].length * 0.75f;

                    grains[j].step++;
                }
            }

            OUT_L[i] = wet;
        }
    }

    grain_start_offset += 0.5f * size;
    if (grain_start_offset >= record_head_pos) {
        grain_start_offset = 0.f;
    }
}

int main(void)
{
    patch.Init();
    button.Init(patch.B7);

    // Clear the buffer
    // std::fill(&buffer[0], &buffer[kBuffSize - 1], 0);

    patch.StartAudio(AudioCallback);

    while(1) {}
} 
