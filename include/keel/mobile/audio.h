#pragma once

#include "keel/array.h"
#include "keel/mobile/types.h"

namespace keel {

// Real-time Microphone PCM Audio Capture and Spectral Analyzer (1024-point Hann FFT).
// Native hardware backends:
// - Android: NDK AAudio (<aaudio/AAudio.h>) low-latency microphone stream
// - Windows: WinMM waveIn PCM capture
// - Fallback / Simulator: Calibrated multi-harmonic + white-noise synthesizer
class AudioCapture {
public:
    static constexpr u32 kDefaultSampleRate = 48000;
    static constexpr usize kWaveformWindowSamples = 4096;
    static constexpr usize kSpectrumBins = 512;

    AudioCapture();
    ~AudioCapture();

    AudioCapture(const AudioCapture&) = delete;
    AudioCapture& operator=(const AudioCapture&) = delete;

    bool initialize(u32 sample_rate = kDefaultSampleRate);
    void shutdown();

    bool set_capturing(bool enabled);
    [[nodiscard]] bool is_capturing() const noexcept { return m_capturing; }
    [[nodiscard]] bool is_using_synthetic_signal() const noexcept { return m_using_synthetic; }
    void set_force_synthetic_signal(bool force_synth) noexcept { m_force_synthetic = force_synth; }

    // Feeds external hardware PCM samples in [-1..+1].
    void push_hardware_samples(ArrayRef<f32> samples);

    // Pulls latest captured PCM samples (or steps synthetic signal) and updates RMS/Peak + 512-bin FFT.
    void update_frame(f64 dt_seconds);

    [[nodiscard]] f32 sample_rate() const noexcept { return static_cast<f32>(m_sample_rate); }

    [[nodiscard]] ArrayRef<f32> waveform_samples() const noexcept { return m_waveform; }
    [[nodiscard]] ArrayRef<f32> spectrum_slice_db() const noexcept { return m_spectrum_db; }
    [[nodiscard]] u64 spectrum_slice_sequence() const noexcept { return m_slice_sequence; }

    [[nodiscard]] f32 rms_dbfs() const noexcept { return m_rms_dbfs; }
    [[nodiscard]] f32 peak_dbfs() const noexcept { return m_peak_dbfs; }
    [[nodiscard]] ArrayRef<f32> stereo_meter_db() const noexcept {
        return ArrayRef<f32>(m_stereo_meter_db, 2);
    }

    [[nodiscard]] f32 dominant_freq_hz() const noexcept { return m_dominant_freq_hz; }
    [[nodiscard]] f32 dominant_peak_db() const noexcept { return m_dominant_peak_db; }

    void set_synth_fundamental_hz(f32 freq_hz) noexcept { m_synth_freq_hz = freq_hz; }
    [[nodiscard]] f32 synth_fundamental_hz() const noexcept { return m_synth_freq_hz; }
    void set_synth_noise_floor_db(f32 noise_db) noexcept { m_synth_noise_db = noise_db; }
    [[nodiscard]] f32 synth_noise_floor_db() const noexcept { return m_synth_noise_db; }

private:
    bool open_hardware_stream();
    void close_hardware_stream();
    bool poll_hardware_stream();
    void compute_metrics_and_spectrum();

    struct NativeAudioState;
    NativeAudioState* m_native = nullptr;

    u32 m_sample_rate = kDefaultSampleRate;
    Array<f32> m_waveform{};
    Array<f32> m_spectrum_db{};
    f32 m_stereo_meter_db[2]{-60.0f, -60.0f};

    u64 m_slice_sequence = 0;
    f32 m_rms_dbfs = -60.0f;
    f32 m_peak_dbfs = -60.0f;
    f32 m_dominant_freq_hz = 440.0f;
    f32 m_dominant_peak_db = -60.0f;

    f32 m_synth_freq_hz = 1000.0f;
    f32 m_synth_noise_db = -52.0f;
    f64 m_synth_phase = 0.0;
    f64 m_sim_time_sec = 0.0;

    bool m_initialized = false;
    bool m_capturing = true;
    bool m_using_synthetic = true;
    bool m_force_synthetic = false;
    bool m_got_external_samples_this_frame = false;
};

} // namespace keel
