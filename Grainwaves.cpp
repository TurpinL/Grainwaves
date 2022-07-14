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

bool isRecording = false;
size_t recordHeadPos = 0;
size_t playHeadPos = 0;

void AudioCallback(
    AudioHandle::InputBuffer  in,
    AudioHandle::OutputBuffer out,
    size_t size
) {
    patch.ProcessAllControls();
    button.Debounce();

    // Toggle the record state on button press
    if(button.RisingEdge())
    {
        if (!isRecording) {
            isRecording = true;
            recordHeadPos = 0;
        } else {
            isRecording = false;
            playHeadPos = 0;
        }
    }

    // Set the led to 5V if the looper is recording
    patch.SetLed(isRecording); 

    // TODO: Fade in/out the ends of the track to prevent pops
    // TODO: Windowing to get rid of pops and squeaks

    // Process audio
    for(size_t i = 0; i < size; i++)
    {
        if (isRecording) {
            buffer[recordHeadPos] = IN_L[i];
            recordHeadPos++;

            if (recordHeadPos >= kBuffSize) {
                isRecording = false;
                playHeadPos = 0;
            }

            OUT_L[i] = IN_L[i];
        } else {
            OUT_L[i] = buffer[playHeadPos] + IN_L[i];
            playHeadPos++;

            if (playHeadPos >= recordHeadPos) {
                playHeadPos = 0;
            }
        }
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
