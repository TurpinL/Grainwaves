#include "daisy_patch_sm.h"
#include "daisysp.h"
#include "core_cm7.h"
#include "Daisy_SSD1327/Daisy_SSD1327.h"
#include "utils.h"
#include "bitmaps.h"

using namespace daisy;
using namespace patch_sm;
using namespace daisysp;
using namespace std;

const int RECORDING_XFADE_OVERLAP = 100; // Samples
const int RECORDING_BUFFER_SIZE = 48000 * 5; // X seconds at 48kHz
const int MIN_GRAIN_SIZE = 480; // 10 ms
const int MAX_GRAIN_SIZE = 48000 * 2; // 2 second
const int MAX_GRAIN_COUNT = 32;
const bool SHOW_PERFORMANCE_BARS = true;
const uint8_t MAX_SPAWN_POINTS = 7;
const int SPAWN_BAR_FLASH_MILLIS = 250;
const int SPAWN_LED_FLASH_MILLIS = 250;
const int SPAWN_TRIGGER_OUT_MILLIS = 2;

struct Grain {
    int length = 0;
    int spawn_position = 0;
    int step = 0;
    float pan = 0; // 0 is left, 1 is right.
    float playback_speed = 0;
};

DaisyPatchSM patch;
CpuLoadMeter cpu_load_meter;

Switch       density_length_link_switch;
Switch       spawn_button;
Switch       record_button;
Switch       play_button;
GPIO         record_led;
GPIO         play_led;

ReverbSc     reverb;

uint8_t DMA_BUFFER_MEM_SECTION oled_buffer[SSD1327_REQUIRED_DMA_BUFFER_SIZE];
Daisy_SSD1327 oled;
SpiHandle spi;

float DSY_SDRAM_BSS recording[RECORDING_BUFFER_SIZE];
size_t recording_length = RECORDING_BUFFER_SIZE;
size_t write_head = 0;

const size_t RENDERABLE_RECORDING_BUFFER_SIZE = oled.width;
const float RECORDING_TO_RENDERABLE_RECORDING_BUFFER_RATIO = RENDERABLE_RECORDING_BUFFER_SIZE / (float)RECORDING_BUFFER_SIZE;
const float RENDERABLE_RECORDING_TO_RECORDING_BUFFER_RATIO = RECORDING_BUFFER_SIZE / (float)RENDERABLE_RECORDING_BUFFER_SIZE;
float DSY_SDRAM_BSS renderable_recording[RENDERABLE_RECORDING_BUFFER_SIZE]; // Much lower resolution, for easy rendering
size_t last_written_renderable_recording_index = 0; 

bool is_tracking = false;
bool is_recording = true;
bool is_stopping_recording = false;
size_t recording_xfade_step = 0;

float spawn_position_scan_speed;
float spawn_position_offset;
float spawn_positions_splay;
float spawn_positions_count;
float pitch_shift_in_semitones;
bool primed_for_manual_spawn = true;
int grain_length;
float grain_density; // Target concurrent grains
unsigned int spawn_time; // The number of samples between each new grain
float actual_spawn_time;
float spawn_time_spread; // The variance of the spawn rate
uint32_t last_spawn_time_at_position[MAX_SPAWN_POINTS] = { 0 };
uint32_t last_spawn_time;
float reverb_wet_mix;

int next_spawn_position_index = 0;
float spawn_position = 0.f;
float next_spawn_offset;
uint32_t samples_since_last_non_manual_spawn = 0;

Grain grains[MAX_GRAIN_COUNT];
Stack<uint8_t, MAX_GRAIN_COUNT> available_grains;

uint32_t last_oled_update_millis = 0;
uint32_t last_debug_print_millis = 0;
uint32_t last_led_update_millis = 0;

void AudioCallback(
    AudioHandle::InputBuffer  in,
    AudioHandle::OutputBuffer out,
    size_t size
);

inline float envelope(float t) {
    return t * t * (3.0f - 2.0f * t);
}

inline size_t get_spawn_position(int index) {
    int unwrapped_spawn_position;
    
    if (is_tracking) {
        unwrapped_spawn_position = write_head + (spawn_position_offset - (recording_length / 2));
    } else {
        unwrapped_spawn_position = spawn_position_offset;
    }
                    
    if (index == (int)spawn_positions_count - 1) {
        unwrapped_spawn_position += spawn_positions_splay;
    } else if (index != 0) {
        unwrapped_spawn_position += index * (spawn_positions_splay / (spawn_positions_count - 2));
    }
    
    return fwrap(unwrapped_spawn_position, 0.f, recording_length);
}

void draw_recorded_waveform() {
    uint8_t last_amplitude = 0;
    // Traversing backwards stops the leading wave of recording
    // affecting values infront of it due to how the smoothing filter works
    for (int x = oled.width - 1; x >= 0; x--) {
        size_t renderable_recording_index = (x / (float)oled.width) * RENDERABLE_RECORDING_BUFFER_SIZE;

        uint8_t amplitude = min(128.f, renderable_recording[renderable_recording_index] / 0.1f * oled.height);

        // Smooth out the waveform
        // TODO: Smooth differently, this produces weird classic LPF shapes
        if (x > 0) {
            amplitude = amplitude * 0.4 + last_amplitude * 0.6;
        }
        last_amplitude = amplitude;

        uint8_t margin = (oled.height - amplitude) / 2;

        for (uint8_t y = margin; y < oled.width - margin; y++) {
            oled.lightenPixel(x, y, 0x1);
        }
    }
}

void draw_write_head_indicator() {
    uint8_t write_head_screen_x = ((write_head / (float)RECORDING_BUFFER_SIZE) * oled.width - 1);
    for (int x = 0; x < write_head_indicator_width; x++) {
        for (int y = 0; y < write_head_indicator_height; y++) {
            uint8_t screen_x = wrap(x + write_head_screen_x - write_head_indicator_width / 2, 0, oled.width);
            uint8_t screen_y = wrap(y + oled.height - write_head_indicator_height, 0, oled.height);
            uint8_t bitmap_index = x + (y * write_head_indicator_width);

            if (is_recording) {
                oled.lightenPixel(screen_x, screen_y, write_head_indicator_recording[bitmap_index]);
            } else {
                oled.lightenPixel(screen_x, screen_y, write_head_indicator_not_recording[bitmap_index]);
            }
        }
    }

    for (int y = (oled.width / 2) - 3; y < (oled.width / 2) + 3; y++) {
        oled.lightenPixel(write_head_screen_x, y, is_recording ? 0x8 : 0x2);
    }
}

void draw_grain_spawn_positions() {
    uint8_t y_margin = 8;

    for (int i = 0; i < (int)spawn_positions_count; i++) {
        float spawn_position_x = get_spawn_position(i) / (float)recording_length * oled.width;
        float spawn_position_x_decimal_part = modf(spawn_position_x);
        int time_since_last_spawn = System::GetNow() - last_spawn_time_at_position[i];
        float flash_intensity = 1 - (min(SPAWN_BAR_FLASH_MILLIS, time_since_last_spawn) / (float)SPAWN_BAR_FLASH_MILLIS);

        uint8_t minColor = flash_intensity * 2;
        uint8_t maxColor = 4 + flash_intensity * 11;

        for (uint8_t y = y_margin; y < oled.height - y_margin; y++) {
            oled.lightenPixel(spawn_position_x, y, map_to_range(1 - spawn_position_x_decimal_part, minColor, maxColor));
            oled.lightenPixel(wrap(spawn_position_x + 1, 0, oled.width), y, map_to_range(spawn_position_x_decimal_part, minColor, maxColor));
        }
    }
}

void draw_grains() {
    for (int j = 0; j < MAX_GRAIN_COUNT; j++) {
        Grain grain = grains[j];

        if (grain.step <= grain.length) {
            uint8_t y = grain.pan * (oled.height - 1);
            uint32_t current_offset = wrap(grain.spawn_position + grain.step * grains[j].playback_speed, 0, recording_length);
            uint8_t x = (current_offset / (float)recording_length) * oled.width;

            float amplitude = 2 * envelope(min((grains[j].length - grains[j].step), grains[j].step) / (float)grains[j].length);  

            oled.lightenPixel(x, y, 0xA * amplitude);
            oled.lightenPixel((x + 1) % oled.width, y, 0xF * amplitude);
            oled.lightenPixel((x + 2) % oled.width, y, 0xA * amplitude);
        }
    }
}

void draw_performance_bars() {
    // CPU Usage
    uint8_t x_max_cpu_load = cpu_load_meter.GetMaxCpuLoad() * oled.width;
    uint8_t x_avg_cpu_load = cpu_load_meter.GetAvgCpuLoad() * oled.width;
    for (int x = 0; x < oled.width; x++) {
        if (x == x_max_cpu_load) {
            oled.lightenPixel(x, 0, 0xF);
            oled.lightenPixel(x, 1, 0xF);
        } else if (x <= x_avg_cpu_load) {
            oled.lightenPixel(x, 0, 0xA);
            oled.lightenPixel(x, 1, 0xA);
        } else {
            oled.lightenPixel(x, 0, 0x0);
            oled.lightenPixel(x, 1, 0x0);
        }
    }

    // Grain count
    uint8_t alive_grains_x = (MAX_GRAIN_COUNT - available_grains.GetNumElements()) / (float)MAX_GRAIN_COUNT * oled.width;
    for (int x = 0; x < oled.width; x++) {
        if (x <= alive_grains_x) {
            oled.lightenPixel(x, 3, 0xA);
            oled.lightenPixel(x, 4, 0xA);
        } else {
            oled.lightenPixel(x, 3, 0x0);
            oled.lightenPixel(x, 4, 0x0);
        }
    }
}

// Responsible for wrapping the index
inline float get_sample(int index) {
    return recording[wrap(index, 0, recording_length)];
}

inline void record_xfaded_sample(float sample_in) {
    float xfade_magnitude = (recording_xfade_step + 1) / ((float)RECORDING_XFADE_OVERLAP + 1.f);

    recording[write_head] = lerp(
        recording[write_head],
        sample_in,
        xfade_magnitude
    );
}

inline void write_spawn_led(float intensity) {
    if (intensity == 0.f) {
        patch.WriteCvOut(CV_OUT_1, 0.f);
    } else {
        patch.WriteCvOut(CV_OUT_1, map_to_range(intensity * intensity, 2.35, 3.75));
    }
}

void process_controls() {
    patch.ProcessAllControls();
    record_button.Debounce();
    play_button.Debounce();
    spawn_button.Debounce();
    density_length_link_switch.Debounce();

    float raw_pitch_pot = patch.GetAdcValue(ADC_9);
    float raw_length_pot = patch.adc.GetMuxFloat(ADC_10, 0);
    float raw_position_pot = patch.adc.GetMuxFloat(ADC_10, 1);
    float raw_jitter_pot = patch.adc.GetMuxFloat(ADC_10, 2);
    float raw_splay_pot = patch.adc.GetMuxFloat(ADC_10, 3);
    float raw_density_pot = patch.adc.GetMuxFloat(ADC_10, 4);
    float raw_count_pot = patch.adc.GetMuxFloat(ADC_10, 5);
    float raw_reverb_pot = patch.adc.GetMuxFloat(ADC_10, 6);
    float raw_spawn_volume_pot = patch.adc.GetMuxFloat(ADC_10, 7);

    float raw_reverb_cv = patch.GetAdcValue(CV_1);
    float raw_pitch_cv = patch.GetAdcValue(CV_2);
    float raw_length_cv = patch.GetAdcValue(CV_3);
    float raw_density_cv = patch.GetAdcValue(CV_4);
    float raw_spawn_cv = patch.GetAdcValue(CV_5);
    float raw_splay_cv = patch.GetAdcValue(CV_6);
    float raw_position_cv = patch.GetAdcValue(CV_7);
    float raw_count_cv = patch.GetAdcValue(CV_8);

    spawn_position_offset = (raw_position_cv + map_to_range(raw_position_pot, 0.99, 0.01)) * recording_length;
    spawn_positions_count = 2.7f + map_to_range(raw_count_pot, 0, MAX_SPAWN_POINTS - 2);

    // Deadzone at 0 to the spawn position splay
    float raw_spawn_pos_splay = map_to_range(raw_splay_pot, 0.5, -0.5) + raw_splay_cv;
    if (raw_spawn_pos_splay < -0.05f) {
        raw_spawn_pos_splay = raw_spawn_pos_splay + 0.05f;
    } else if (raw_spawn_pos_splay > 0.05f) {
        raw_spawn_pos_splay = raw_spawn_pos_splay - 0.05f;
    } else {
        raw_spawn_pos_splay = 0;
        spawn_positions_count = 1;
    }
    spawn_positions_splay = raw_spawn_pos_splay * recording_length;

    // pitch_shift_in_semitones = map_to_range(patch.GetAdcValue(CV_7), -12 * 5, 12 * 5); // volt per octave
    pitch_shift_in_semitones = map_to_range(raw_pitch_pot, -24, 24) + raw_pitch_cv * 5 * 12; // Without CV this is more playable

    grain_length = map_to_range(pow(abs(raw_length_pot - 0.5f) * 2, 2), MIN_GRAIN_SIZE, MAX_GRAIN_SIZE);
    if (raw_length_pot < 0.5f) {
        grain_length = -grain_length;
    }

    if (density_length_link_switch.Pressed()) {
        spawn_time = map_to_range(1 - log10f(1 + raw_density_pot * 9), 0, MAX_GRAIN_SIZE / 4);
    } else {
        grain_density = map_to_range(pow(raw_density_pot, 2), 0.5f, MAX_GRAIN_COUNT);
        spawn_time = abs(grain_length) / grain_density;
    }

    spawn_time_spread = raw_jitter_pot;
    if (raw_density_pot <= 0.001) {
        actual_spawn_time = INFINITY;
    } else {
        actual_spawn_time = spawn_time * (1 + next_spawn_offset * spawn_time_spread);
    }

    float reverb_amount = raw_reverb_pot;
    float reverb_time = fmap(min(reverb_amount, 0.5f) * 2, 0.5f, 0.99f);
    float reverb_damp = fmap(reverb_amount, 100.f, 24000.f, Mapping::LOG);
    reverb_wet_mix = min(reverb_amount, 0.1f) * 7;

    reverb.SetFeedback(reverb_time);
    reverb.SetLpFreq(reverb_damp);

    // Record button/gate
    if(record_button.RisingEdge() || patch.gate_in_1.Trig())
    {
        record_led.Write(!is_recording);

        if (!is_recording) {
            is_recording = true;
            recording_xfade_step = 0;
        } else {
            is_stopping_recording = true;
        }
    }

    // Play button/gate
    if (play_button.RisingEdge() || patch.gate_in_2.Trig()) {
        is_tracking = !is_tracking;
        play_led.Write(is_tracking);
    }

    // Spawn button
    if (spawn_button.FallingEdge()) {
        primed_for_manual_spawn = true;
    }

    // Spawn trigger out
    int time_since_last_spawn = System::GetNow() - last_spawn_time;
    dsy_gpio_write(&patch.gate_out_2, time_since_last_spawn < SPAWN_TRIGGER_OUT_MILLIS);
}

void spawn_grain() {
    size_t new_grain_index = available_grains.PopBack();

    grains[new_grain_index].length = abs(grain_length);
    grains[new_grain_index].step = 0;
    grains[new_grain_index].pan = 0.5f + randF(-0.5f, 0.5f);
    
    grains[new_grain_index].spawn_position = get_spawn_position(next_spawn_position_index);
    last_spawn_time_at_position[next_spawn_position_index] = System::GetNow();
    last_spawn_time = System::GetNow();

    float pitch_shift_in_octaves = pitch_shift_in_semitones / 12.f;

    // Reverse the playback if the length is negative
    if (grain_length > 0) {
        grains[new_grain_index].playback_speed = pow(2, pitch_shift_in_octaves);
    } else {
        grains[new_grain_index].playback_speed = -pow(2, pitch_shift_in_octaves);
    }

    next_spawn_offset = randF(-1.f, 1.f); // +/- 100%
    next_spawn_position_index = wrap(next_spawn_position_index + 1, 0, (int)spawn_positions_count);
}

void record_sample(float sample) {
    if (is_stopping_recording) {
        // Record a little extra at the end of the recording so we can xfade the values
        // and stop the pop sound
        record_xfaded_sample(sample);

        recording_xfade_step--;

        if (recording_xfade_step == 0) {
            is_recording = false;
            is_stopping_recording = false;
        }
    } else if (recording_xfade_step < RECORDING_XFADE_OVERLAP) {
        // xfade in the start of the recording to stop the pop sound
        record_xfaded_sample(sample);

        recording_xfade_step++;
    } else {
        recording[write_head] = sample;
    }
}

void record_sample_for_display(float sample) {
    // TODO: Record positive and negative values seperately
    size_t renderable_recording_index = write_head * RECORDING_TO_RENDERABLE_RECORDING_BUFFER_RATIO;

    // Clear out the element when we first start writing fresh values to it
    if (write_head == 0 || renderable_recording_index > last_written_renderable_recording_index) {
        renderable_recording[renderable_recording_index] = 0;
    }

    // Downsample the samples into renderable_recording_index by averaging them
    renderable_recording[renderable_recording_index] += abs(sample) / RENDERABLE_RECORDING_TO_RECORDING_BUFFER_RATIO;
    last_written_renderable_recording_index = renderable_recording_index;
    
    if (recording_length < RECORDING_BUFFER_SIZE) {
        recording_length++;
    }
}

void increment_write_head() {
    write_head++;

    if (write_head >= RECORDING_BUFFER_SIZE) {
        write_head = 0;
    }
}

void calculate_audio_out(float in_l, float in_r, float &out_l, float &out_r) {
    float wet_l = 0.f;
    float wet_r = 0.f;

    for (int j = 0; j < MAX_GRAIN_COUNT; j++) {
        if (grains[j].step <= grains[j].length) {
            size_t buffer_index = grains[j].spawn_position + grains[j].step * grains[j].playback_speed;

            // playback_speed is a float so we need to interpolate between samples
            float sample = get_sample(buffer_index);
            float next_sample = get_sample(buffer_index + 1);

            float decimal_portion = modf(grains[j].step * grains[j].playback_speed);
            float interpolated_sample = sample * (1 - decimal_portion) + next_sample * decimal_portion;

            float envelope_progress = min((grains[j].length - grains[j].step), grains[j].step) / max(1.f, (float)grains[j].length);
            float signal = interpolated_sample * envelope(envelope_progress);

            wet_l += (1.f - grains[j].pan) * signal;
            wet_r += grains[j].pan * signal;

            grains[j].step++;

            if (grains[j].step > grains[j].length) {
                available_grains.PushBack(j);
            }
        }
    }

    float reverb_in_l = wet_l;
    float reverb_in_r = wet_r;
    float reverb_wet_l, reverb_wet_r;
    reverb.Process(reverb_in_l, reverb_in_r, &reverb_wet_l, &reverb_wet_r);
    
    out_l = reverb_in_l + (reverb_wet_l * reverb_wet_mix) + in_l;
    out_r = reverb_in_r + (reverb_wet_r * reverb_wet_mix) + in_r;
}

void init() {
    patch.Init();
    patch.StartLog();

    // Init GPIO
    density_length_link_switch.Init(patch.D5, 0, Switch::TYPE_TOGGLE, Switch::POLARITY_NORMAL, Switch::PULL_NONE);
    record_button.Init(patch.D6, 0, Switch::TYPE_MOMENTARY, Switch::POLARITY_NORMAL, Switch::PULL_NONE);
    play_button.Init(patch.D7, 0, Switch::TYPE_MOMENTARY, Switch::POLARITY_NORMAL, Switch::PULL_NONE);
    spawn_button.Init(patch.A8, 0, Switch::TYPE_MOMENTARY, Switch::POLARITY_NORMAL, Switch::PULL_NONE);
    
    record_led.Init(patch.B7, GPIO::Mode::OUTPUT);
    record_led.Write(is_recording);

    play_led.Init(patch.B8, GPIO::Mode::OUTPUT);
    play_led.Write(is_tracking);


    // Init OLED
    spi.Init(
        oled.getSpiConfig(
            patch.D10, /* sclk */
            patch.A9, /* mosi */ // Using A9 instead of D9 because I fried D9
            patch.D8, /* miso */
            patch.D1 /* nss */
        )
    );

    oled.init(spi, patch.D8, oled_buffer, patch);
    oled.clear(0x5);
    oled.display();
    
    // Populate available grains stack
    for (u_int8_t i = 0; i < MAX_GRAIN_COUNT; i++) {
        available_grains.PushBack(i);
    }

    // Clear the buffers
    memset(renderable_recording, 0, sizeof(renderable_recording));
    memset(recording, 0, sizeof(recording));

    cpu_load_meter.Init(patch.AudioSampleRate(), patch.AudioBlockSize());
    reverb.Init(patch.AudioSampleRate());
    patch.StartAudio(AudioCallback);
}

void log_debug_info() {
    last_debug_print_millis = System::GetNow();

    // Note, this ignores any work done in this loop, eg running the OLED
    // patch.PrintLine("cpu Max: " FLT_FMT3 " Avg:" FLT_FMT3, FLT_VAR3(cpu_load_meter.GetMaxCpuLoad()), FLT_VAR3(cpu_load_meter.GetAvgCpuLoad()));
    // patch.PrintLine(FLT_FMT3, FLT_VAR3(patch.adc.GetMuxFloat(ADC_10, 4)));
    patch.PrintLine("%d", patch.adc.GetMuxFloat(ADC_10, 4) <= 0.001);
    // patch.PrintLine(FLT_FMT3 ", " FLT_FMT3 ", " FLT_FMT3 ", " FLT_FMT3 ", " FLT_FMT3 ", " FLT_FMT3 ", " FLT_FMT3 ", " FLT_FMT3 ", " FLT_FMT3 ", ", 
    //     FLT_VAR3(patch.adc.GetMuxFloat(ADC_10, 0)), 
    //     FLT_VAR3(patch.adc.GetMuxFloat(ADC_10, 1)),
    //     FLT_VAR3(patch.adc.GetMuxFloat(ADC_10, 2)),
    //     FLT_VAR3(patch.adc.GetMuxFloat(ADC_10, 3)),
    //     FLT_VAR3(patch.adc.GetMuxFloat(ADC_10, 4)),
    //     FLT_VAR3(patch.adc.GetMuxFloat(ADC_10, 5)),
    //     FLT_VAR3(patch.adc.GetMuxFloat(ADC_10, 6)),
    //     FLT_VAR3(patch.adc.GetMuxFloat(ADC_10, 7)),
    //     FLT_VAR3(patch.GetAdcValue(ADC_9))
    // );

    // patch.PrintLine(FLT_FMT3 ", " FLT_FMT3 ", " FLT_FMT3 ", " FLT_FMT3 ", " FLT_FMT3 ", " FLT_FMT3 ", " FLT_FMT3 ", " FLT_FMT3, 
    //     FLT_VAR3(patch.GetAdcValue(CV_1)),
    //     FLT_VAR3(patch.GetAdcValue(CV_2)),
    //     FLT_VAR3(patch.GetAdcValue(CV_3)),
    //     FLT_VAR3(patch.GetAdcValue(CV_4)),
    //     FLT_VAR3(patch.GetAdcValue(CV_5)),
    //     FLT_VAR3(patch.GetAdcValue(CV_6)),
    //     FLT_VAR3(patch.GetAdcValue(CV_7)),
    //     FLT_VAR3(patch.GetAdcValue(CV_8))
    // );
}

void AudioCallback(
    AudioHandle::InputBuffer  in,
    AudioHandle::OutputBuffer out,
    size_t size
) {
    cpu_load_meter.OnBlockStart();

    process_controls();

    // Process audio output
    for(size_t i = 0; i < size; i++)
    {
        if (is_recording) {
            record_sample(IN_L[i]);
            record_sample_for_display(IN_L[i]);
        }

        // Progresses the write head regardless of if we're recording
        increment_write_head(); 

        // Workout if we need to spawn a grain this sample
        samples_since_last_non_manual_spawn++;

        bool has_spawn_timer_elapsed = actual_spawn_time != INFINITY 
                && samples_since_last_non_manual_spawn >= actual_spawn_time;
        bool should_manual_spawn = spawn_button.RisingEdge() && primed_for_manual_spawn;

        // Spawn grains
        if ((has_spawn_timer_elapsed || should_manual_spawn) && !available_grains.IsEmpty()) {
            spawn_grain();

            if (has_spawn_timer_elapsed) {
                samples_since_last_non_manual_spawn = 0;
            }

            if (should_manual_spawn) {
                primed_for_manual_spawn = false;
            }
        }

        calculate_audio_out(IN_L[i], IN_L[i], OUT_L[i], OUT_R[i]);
    }

    cpu_load_meter.OnBlockEnd();
}

int main(void)
{
    init();
    patch.SetLed(true); // Turn on LED when init is complete

    while(1) {
        // Draw to oled
        if (System::GetNow() - last_oled_update_millis > 8 && !oled.isRendering()) {
            last_oled_update_millis = System::GetNow();

            oled.clear(SSD1327_BLACK);
            draw_recorded_waveform();
            draw_write_head_indicator();
            draw_grain_spawn_positions();
            draw_grains();
            if (SHOW_PERFORMANCE_BARS) { draw_performance_bars(); }
            oled.display();
        }

        // LEDs
        if (System::GetNow() - last_led_update_millis > 8) {
            // Spawn LED
            int time_since_last_spawn = System::GetNow() - last_spawn_time;
            float spawn_led_intensity = 1 - (min(SPAWN_LED_FLASH_MILLIS, time_since_last_spawn) / (float)SPAWN_LED_FLASH_MILLIS);
            write_spawn_led(spawn_led_intensity * 0.5f);
        }

        // Debug logs
        if (System::GetNow() - last_debug_print_millis > 50) {
            log_debug_info();
        }
    }
}