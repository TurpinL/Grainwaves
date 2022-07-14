#include "daisy_patch_sm.h"
#include "daisysp.h"

using namespace daisy;
using namespace patch_sm;
using namespace daisysp;

DaisyPatchSM patch;
Switch       button;

#define kBuffSize 48000 * 3 // 3 seconds at 48kHz

// Loopers and the buffers they'll use
float DSY_SDRAM_BSS buffer[kBuffSize];

bool is_recording = false;
size_t record_head_pos = 0;
int grain_length = 48000 / 5; 
int grain_progress = 0;
float grain_start_offset = 0.f;
int grain_count = 1;
int count = 0;

void AudioCallback(
    AudioHandle::InputBuffer  in,
    AudioHandle::OutputBuffer out,
    size_t size
) {
    patch.ProcessAllControls();
    button.Debounce();

    // if (count % 200 == 0) {
        grain_count = patch.GetAdcValue(CV_1) * 10 + 1;
        grain_length = patch.GetAdcValue(CV_2) * 48000;
    // }
    
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
        if (is_recording) {
            buffer[record_head_pos] = IN_L[i];
            record_head_pos++;

            if (record_head_pos >= kBuffSize) {
                is_recording = false;
            }

            OUT_L[i] = IN_L[i];
        } else {
            float wet = 0.f;

            for (int j = 0; j < grain_count; j++) {
                size_t grain_offset = (3200 * j + grain_progress) % grain_length;
                size_t grain_pos = ((size_t)grain_start_offset + grain_offset) % record_head_pos;

                wet += buffer[grain_pos];
            }

            OUT_L[i] = wet;
            grain_progress++;

            if (grain_progress >= grain_length) {
                grain_progress = 0;
            }
        }
    }

    count++;
    grain_start_offset += 0.02 * size;
    if (grain_start_offset >= std::max((int)record_head_pos - grain_length, 0)) {
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
