#include "keel/mobile/audio.h"
#include "keel/algo.h"

#include <cmath>
#include <numbers>

namespace keel {

namespace {

struct ComplexF32 {
    f32 re = 0.0f;
    f32 im = 0.0f;

    [[nodiscard]] constexpr ComplexF32 operator+(const ComplexF32& b) const noexcept {
        return {re + b.re, im + b.im};
    }

    [[nodiscard]] constexpr ComplexF32 operator-(const ComplexF32& b) const noexcept {
        return {re - b.re, im - b.im};
    }

    [[nodiscard]] constexpr ComplexF32 operator*(const ComplexF32& b) const noexcept {
        return {re * b.re - im * b.im, re * b.im + im * b.re};
    }

    [[nodiscard]] f32 abs() const noexcept {
        return std::sqrt(re * re + im * im);
    }
};

// In-place radix-2 Cooley-Tukey FFT for real-time 1024-point spectrum computation.
void radix2_fft(ComplexF32* a, usize n) noexcept {
    if (n <= 1) return;

    for (usize i = 1, j = 0; i < n; ++i) {
        usize bit = n >> 1;
        for (; j & bit; bit >>= 1) {
            j ^= bit;
        }
        j ^= bit;
        if (i < j) {
            keel::swap(a[i], a[j]);
        }
    }

    for (usize len = 2; len <= n; len <<= 1) {
        const f32 angle = -2.0f * std::numbers::pi_v<f32> / static_cast<f32>(len);
        const ComplexF32 wlen{std::cos(angle), std::sin(angle)};
        for (usize i = 0; i < n; i += len) {
            ComplexF32 w{1.0f, 0.0f};
            for (usize j = 0; j < len / 2; ++j) {
                const ComplexF32 u = a[i + j];
                const ComplexF32 v = a[i + j + len / 2] * w;
                a[i + j] = u + v;
                a[i + j + len / 2] = u - v;
                w = w * wlen;
            }
        }
    }
}

} // namespace

AudioCapture::AudioCapture() {
    m_waveform.resize(kWaveformWindowSamples, 0.0f);
    m_spectrum_db.resize(kSpectrumBins, -95.0f);
}

AudioCapture::~AudioCapture() {
    shutdown();
}

bool AudioCapture::initialize(u32 sample_rate) {
    if (m_initialized) return true;

    m_sample_rate = sample_rate > 0 ? sample_rate : kDefaultSampleRate;
    m_waveform.clear();
    m_waveform.resize(kWaveformWindowSamples, 0.0f);
    m_spectrum_db.clear();
    m_spectrum_db.resize(kSpectrumBins, -95.0f);

    m_initialized = true;
    m_capturing = true;
    m_using_synthetic = true;
    update_frame(0.016);
    return true;
}

void AudioCapture::shutdown() {
    m_initialized = false;
    m_capturing = false;
}

bool AudioCapture::set_capturing(bool enabled) {
    m_capturing = enabled;
    return true;
}

void AudioCapture::push_hardware_samples(ArrayRef<f32> samples) {
    if (samples.empty() || m_force_synthetic) return;
    if (m_waveform.size() < kWaveformWindowSamples) {
        m_waveform.resize(kWaveformWindowSamples, 0.0f);
    }

    const usize count = samples.size();
    if (count >= m_waveform.size()) {
        mem_copy(
            m_waveform.data(),
            samples.data() + (count - m_waveform.size()),
            m_waveform.size() * sizeof(f32)
        );
    } else {
        const usize keep = m_waveform.size() - count;
        mem_move(m_waveform.data(), m_waveform.data() + count, keep * sizeof(f32));
        mem_copy(m_waveform.data() + keep, samples.data(), count * sizeof(f32));
    }

    m_using_synthetic = false;
    m_got_external_samples_this_frame = true;
}

void AudioCapture::update_frame(f64 dt_seconds) {
    if (!m_capturing) return;

    m_sim_time_sec += dt_seconds;

    if (!m_got_external_samples_this_frame && (m_using_synthetic || m_force_synthetic)) {
        m_using_synthetic = true;
        const usize new_samples = keel::clamp<usize>(
            static_cast<usize>(std::round(dt_seconds * m_sample_rate)),
            128,
            1024
        );
        const usize keep = m_waveform.size() - new_samples;
        mem_move(m_waveform.data(), m_waveform.data() + new_samples, keep * sizeof(f32));

        const f64 two_pi = 2.0 * std::numbers::pi;
        const f64 f0 = keel::clamp(static_cast<f64>(m_synth_freq_hz), 60.0, 18000.0);
        const f64 phase_step = two_pi * f0 / static_cast<f64>(m_sample_rate);
        const f32 noise_amp = std::pow(10.0f, m_synth_noise_db / 20.0f);
        const f32 am_mod = 0.65f + 0.25f * std::sin(static_cast<f32>(m_sim_time_sec * 2.2));

        u32 rng = static_cast<u32>(m_slice_sequence * 1664525u + 1013904223u);
        for (usize i = keep; i < m_waveform.size(); ++i) {
            rng = rng * 1664525u + 1013904223u;
            const f32 white = (static_cast<f32>(rng & 0xFFFFu) / 32768.0f) - 1.0f;

            const f32 s1 = 0.48f * am_mod * static_cast<f32>(std::sin(m_synth_phase));
            const f32 s2 = 0.14f * static_cast<f32>(std::sin(m_synth_phase * 2.0));
            const f32 s3 = 0.06f * static_cast<f32>(std::sin(m_synth_phase * 3.0));
            m_waveform[i] = keel::clamp(s1 + s2 + s3 + white * noise_amp, -1.0f, 1.0f);

            m_synth_phase += phase_step;
            if (m_synth_phase > two_pi) m_synth_phase -= two_pi;
        }
    }

    m_got_external_samples_this_frame = false;
    compute_metrics_and_spectrum();
}

void AudioCapture::compute_metrics_and_spectrum() {
    constexpr usize kFftSize = kSpectrumBins * 2; // 1024-point FFT -> 512 positive bins
    if (m_waveform.size() < kFftSize) return;

    const f32* window_start = m_waveform.data() + (m_waveform.size() - kFftSize);

    f64 sum_sq = 0.0;
    f32 peak = 0.0f;
    ComplexF32 fft_buf[kFftSize]{};

    for (usize i = 0; i < kFftSize; ++i) {
        const f32 s = window_start[i];
        sum_sq += static_cast<f64>(s) * s;
        peak = keel::max(peak, std::abs(s));

        const f32 w = 0.5f * (1.0f - std::cos(2.0f * std::numbers::pi_v<f32> * static_cast<f32>(i) / static_cast<f32>(kFftSize - 1)));
        fft_buf[i] = {s * w, 0.0f};
    }

    const f32 rms = static_cast<f32>(std::sqrt(sum_sq / static_cast<f64>(kFftSize)));
    m_rms_dbfs = keel::clamp(20.0f * std::log10(keel::max(rms, 1e-5f)), -60.0f, 0.0f);
    m_peak_dbfs = keel::clamp(20.0f * std::log10(keel::max(peak, 1e-5f)), -60.0f, 0.0f);
    m_stereo_meter_db[0] = m_rms_dbfs;
    m_stereo_meter_db[1] = keel::clamp(m_rms_dbfs - 1.2f, -60.0f, 0.0f);

    radix2_fft(fft_buf, kFftSize);

    const f32 inv_norm = 2.0f / static_cast<f32>(kFftSize);
    f32 max_bin_db = -120.0f;
    usize max_bin_idx = 1;

    for (usize k = 0; k < kSpectrumBins; ++k) {
        const f32 mag = fft_buf[k].abs() * inv_norm;
        const f32 db = keel::clamp(20.0f * std::log10(keel::max(mag, 1e-6f)), -110.0f, 0.0f);
        m_spectrum_db[k] = db;
        if (k > 1 && db > max_bin_db) {
            max_bin_db = db;
            max_bin_idx = k;
        }
    }

    m_dominant_peak_db = max_bin_db;
    m_dominant_freq_hz = static_cast<f32>(max_bin_idx) * (static_cast<f32>(m_sample_rate) * 0.5f) / static_cast<f32>(kSpectrumBins);
    ++m_slice_sequence;
}

} // namespace keel
