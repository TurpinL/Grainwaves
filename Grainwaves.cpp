#include "daisy_patch_sm.h"
#include "daisysp.h"
#include "core_cm7.h"
#include "Daisy_SSD1327/Daisy_SSD1327.h"

using namespace daisy;
using namespace patch_sm;
using namespace daisysp;

DaisyPatchSM patch;
CpuLoadMeter cpuLoadMeter;
Switch       record_button;
Switch       shift_button;

uint8_t DMA_BUFFER_MEM_SECTION oled_buffer[SSD1327_REQUIRED_DMA_BUFFER_SIZE];
Daisy_SSD1327 oled;
SpiHandle spi;
dsy_gpio dc_pin;
I2CHandle i2c;

#define kBuffSize 48000 * 10 // X seconds at 48kHz
#define min_grain_size 480 // 10 ms
#define max_grain_size 48000 // 1 second
#define max_grain_count 64

struct Grain {
    size_t length = 0;
    size_t start_offset = 0;
    size_t step = 0;
    float pan = 0; // 0 is left, 1 is right
    float playback_speed = 0;
};

// Loopers and the buffers they'll use
float DSY_SDRAM_BSS buffer[kBuffSize];

bool is_recording = false;
size_t recording_length = 0;
size_t grain_length = 48000 / 5; 
float grain_start_offset = 0.f;
unsigned int spawn_rate = 48000 / 3; // samples
float spawn_rate_spread;
float next_spawn_offset;
float pan_spread;
uint32_t last_spawn_time = 0;
uint32_t samples_seen = 0;
Grain grains[max_grain_count];
Stack<uint8_t, max_grain_count> available_grains;
uint32_t cycles_used = 0;

float fwrap(float x, float min, float max) {
    if (max == min) return min;
    if (min > max) return fwrap(x, max, min);

    return (x >= 0 ? min : max) + fmodf(x, max - min);
}

int wrap(int x, int min, int max) {
    if (max == min) return min;
    if (min > max) return wrap(x, max, min);

    return (x >= 0 ? min : max) + x % (max - min);
}

float randF(float min, float max) {
    return min + rand() * kRandFrac * (max - min);
}

float mapToRange(float fraction, float min, float max) {
    return min + fraction * (max - min);
}

float modf(float x) {
    static float junk;
    return modf(x, &junk);
}

void AudioCallback(
    AudioHandle::InputBuffer  in,
    AudioHandle::OutputBuffer out,
    size_t size
) {
    cpuLoadMeter.OnBlockStart();

    patch.ProcessAllControls();
    record_button.Debounce();
    shift_button.Debounce();

    spawn_rate = mapToRange(1 - patch.GetAdcValue(CV_1), min_grain_size, max_grain_size);
    grain_length = mapToRange(patch.GetAdcValue(CV_2), min_grain_size, max_grain_size);
    float scan_speed = mapToRange(patch.GetAdcValue(CV_3), -2, 2);
    float grain_spread = patch.GetAdcValue(CV_4);
    pan_spread = patch.GetAdcValue(CV_5);
    float spawn_rate_spread = patch.GetAdcValue(CV_6);
    float modified_spawn_rate = spawn_rate * (1 + next_spawn_offset * spawn_rate_spread);
    float pitch_shift_in_semitones = mapToRange(patch.GetAdcValue(CV_7), -12 * 5, 12 * 5); // volt per octave
    float pitch_shift_spread = patch.GetAdcValue(CV_8);

    // Toggle the record state on button press
    if(record_button.RisingEdge())
    {
        if (!is_recording) {
            is_recording = true;
            recording_length = 0;
        } else {
            is_recording = false;
        }
    }

    // Set the led to 5V if the looper is recording
    patch.SetLed(is_recording || shift_button.Pressed()); 

    // TODO: Fade in/out the ends of the track to prevent pops
    // TODO: Windowing to get rid of pops and squeaks

    // Process audio
    for(size_t i = 0; i < size; i++)
    {
        samples_seen++;

        if (is_recording) {
            buffer[recording_length] = IN_L[i];
            recording_length++;

            if (recording_length >= kBuffSize) {
                is_recording = false;
            }
 
            OUT_L[i] = IN_L[i];
            OUT_R[i] = OUT_L[i];
        } else if (recording_length == 0) {
            // Just pass through if there's no recording
            OUT_L[i] = IN_L[i];
            OUT_R[i] = OUT_L[i];
        } else {
            // Spawn grains
            // TODO: This will break when samples_seen wraps
            if (samples_seen - last_spawn_time >= modified_spawn_rate && !available_grains.IsEmpty()) {
                size_t next_grain_to_spawn = available_grains.PopBack();

                last_spawn_time = samples_seen;

                grains[next_grain_to_spawn].length = grain_length;
                grains[next_grain_to_spawn].start_offset = wrap(grain_start_offset + grain_spread * randF(-0.5f, 0.5f) * recording_length, 0, recording_length);
                grains[next_grain_to_spawn].step = 0;
                grains[next_grain_to_spawn].pan = 0.5f + randF(-0.5f, 0.5f) * pan_spread;

                float pitch_shift_offset_in_semitones = randF(-2.f, 2.f) * pitch_shift_spread;
                float pitch_shift_in_octaves = (pitch_shift_in_semitones + pitch_shift_offset_in_semitones) / 12.f;
                float playback_speed = pow(2, pitch_shift_in_octaves);
                grains[next_grain_to_spawn].playback_speed = playback_speed;

                next_spawn_offset = randF(-1.f, 1.f);
            }

            // Calculate output
            float wetL = 0.f;
            float wetR = 0.f;

            for (int j = 0; j < max_grain_count; j++) {
                if (grains[j].step <= grains[j].length) {
                    size_t buffer_index = wrap(grains[j].start_offset + grains[j].step * grains[j].playback_speed, 0, recording_length);

                    // playback_speed is a float so we need to interpolate between samples
                    float sample = buffer[buffer_index];
                    float nextSample = buffer[wrap(buffer_index + 1, 0, recording_length)];

                    float decimal_portion = modf(grains[j].step * grains[j].playback_speed);
                    float interpolated_sample = sample * (1 - decimal_portion) + nextSample * decimal_portion;

                    // hacky bad envelope
                    float envelope_mult = std::min((grains[j].length - grains[j].step), grains[j].step);
                    // TODO: Get rid of this 0.75f and balance the output properly
                    float signal = interpolated_sample * envelope_mult / grains[j].length * 0.75f;
                    wetL += (1.f - grains[j].pan) * signal;
                    wetR += grains[j].pan * signal;

                    grains[j].step++;

                    if (grains[j].step > grains[j].length) {
                        available_grains.PushBack(j);
                    }
                }
            }

            OUT_L[i] = wetL + IN_L[i];
            OUT_R[i] = wetR + IN_L[i];
        }
    }

    grain_start_offset = fwrap(grain_start_offset + scan_speed * size, 0.f, recording_length);

    cpuLoadMeter.OnBlockEnd();
}

uint32_t last_render_millis = 0;
uint32_t last_debug_print_millis = 0;

int main(void)
{
    patch.Init();
    record_button.Init(patch.B7);
    shift_button.Init(patch.B8);
    patch.StartLog();

    spi.Init(
        oled.getSpiConfig(
            patch.D10, /* sclk */
            patch.D9, /* mosi */
            patch.D8, /* miso */
            patch.D1 /* nss */
        )
    );

    oled.init(spi, patch.A9, oled_buffer, patch);
    oled.clear(0x5);
    oled.display();

    // Populate available grains stack
    for (u_int8_t i = 0; i < max_grain_count; i++) {
        available_grains.PushBack(i);
    }

    cpuLoadMeter.Init(patch.AudioSampleRate(), patch.AudioBlockSize());
    patch.StartAudio(AudioCallback);

    while(1) {
        if (System::GetNow() - last_render_millis > 8 && !oled.isRendering()) {
            last_render_millis = System::GetNow();

            if (recording_length == 0) {
                oled.clear(0x2);
            } else {
                oled.clear(SSD1327_BLACK);
            }

            // Grain start offset
            float grain_start_offset_y = grain_start_offset / (float)recording_length * oled.height;
            float grain_start_offset_y_decimal_part = modf(grain_start_offset_y);
            uint8_t x_margin = (oled.width - (pan_spread * oled.width)) / 2;

            for (uint8_t x = x_margin; x < oled.width - x_margin; x++) {
                oled.setPixel(x, grain_start_offset_y, mapToRange(1 - grain_start_offset_y_decimal_part, 0x2, 0x6));
                oled.setPixel(x, wrap(grain_start_offset_y + 1, 0, oled.height), mapToRange(grain_start_offset_y_decimal_part, 0x2, 0x6));
            }

            // Grains
            for (int j = 0; j < max_grain_count; j++) {
                Grain grain = grains[j];

                if (grain.step <= grain.length) {
                    uint8_t x = grain.pan * oled.width;
                    uint32_t currentOffset = wrap(grain.start_offset + grain.step * grains[j].playback_speed, 0, recording_length);
                    float y = (currentOffset / (float)recording_length) * oled.height;

                    float amplitude = std::min((grains[j].length - grains[j].step), grains[j].step) / (float)grains[j].length;

                    oled.setPixel(x, y, 0xF * amplitude);
                }
            }

            oled.display();
        }

        if (System::GetNow() - last_debug_print_millis > 250) {
            last_debug_print_millis = System::GetNow();

            // Note, this ignores any work done in this loop, eg running the OLED
            patch.PrintLine("cpu Max:" FLT_FMT3 "\tAvg:" FLT_FMT3, FLT_VAR3(cpuLoadMeter.GetMaxCpuLoad()), FLT_VAR3(cpuLoadMeter.GetAvgCpuLoad()));
            // patch.PrintLine(FLT_FMT3, FLT_VAR3(round(mapToRange(patch.GetAdcValue(CV_7), -12, 12)) / 12));
            // patch.PrintLine("grain_start_offset: " FLT_FMT3, FLT_VAR3(grain_start_offset));
        }
    }
} 
 