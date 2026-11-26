#include "keel/mobile/audio.h"
#include "keel/algo.h"

#include "keel/math.h"
#include "keel/sync.h"

#if defined(__ANDROID__)
#include <aaudio/AAudio.h>
#include <android/log.h>
#elif defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <mmsystem.h>
#endif

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
        return keel::sqrt(re * re + im * im);
    }
};

// In-place radix-2 Cooley-Tukey FFT for real-time 1024-point spectrum computation.
void radix2_fft(ComplexF32* a, usize n) noexcept {
    if (!a || n <= 1 || (n & (n - 1u)) != 0) return;

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
        const f32 angle = -2.0f * keel::as<f32>(keel::kPi) / static_cast<f32>(len);
        const ComplexF32 wlen{keel::cos(angle), keel::sin(angle)};
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

struct AudioCapture::NativeAudioState {
#if defined(__ANDROID__)
    AAudioStream* stream = nullptr;
    keel::Mutex mutex;
    Array<f32> staging;

    static aaudio_data_callback_result_t on_audio_ready(
        AAudioStream* /*stream*/,
        void* user_data,
        void* audio_data,
        i32 num_frames
    ) {
        auto* self = static_cast<NativeAudioState*>(user_data);
        if (!self || !audio_data || num_frames <= 0) {
            return AAUDIO_CALLBACK_RESULT_CONTINUE;
        }
        const auto* samples = static_cast<const f32*>(audio_data);
        const usize count = static_cast<usize>(num_frames);
        keel::LockGuard<keel::Mutex> lock(self->mutex);
        if (self->staging.size() + count > AudioCapture::kWaveformWindowSamples * 2) {
            self->staging.clear();
        }
        const usize old_sz = self->staging.size();
        self->staging.resize(old_sz + count);
        mem_copy(self->staging.data() + old_sz, samples, count * sizeof(f32));
        return AAUDIO_CALLBACK_RESULT_CONTINUE;
    }
#elif defined(_WIN32)
    static constexpr usize kNumBuffers = 4;
    static constexpr usize kBufferSamples = 1024;
    HWAVEIN wave_in = nullptr;
    WAVEHDR headers[kNumBuffers]{};
    i16 pcm_buffers[kNumBuffers][kBufferSamples]{};
    bool headers_prepared[kNumBuffers]{};
#endif
};

AudioCapture::AudioCapture() {
    m_waveform.resize(kWaveformWindowSamples, 0.0f);
    m_spectrum_db.resize(kSpectrumBins, -95.0f);
}

AudioCapture::~AudioCapture() {
    shutdown();
}

bool AudioCapture::open_hardware_stream() {
    close_hardware_stream();
    m_native = new NativeAudioState();

#if defined(__ANDROID__)
    AAudioStreamBuilder* builder = nullptr;
    if (AAudio_createStreamBuilder(&builder) != AAUDIO_OK || !builder) {
        close_hardware_stream();
        return false;
    }

    AAudioStreamBuilder_setDirection(builder, AAUDIO_DIRECTION_INPUT);
    AAudioStreamBuilder_setSampleRate(builder, static_cast<i32>(m_sample_rate));
    AAudioStreamBuilder_setChannelCount(builder, 1);
    AAudioStreamBuilder_setFormat(builder, AAUDIO_FORMAT_PCM_FLOAT);
    AAudioStreamBuilder_setPerformanceMode(builder, AAUDIO_PERFORMANCE_MODE_LOW_LATENCY);
    AAudioStreamBuilder_setSharingMode(builder, AAUDIO_SHARING_MODE_SHARED);
    AAudioStreamBuilder_setDataCallback(builder, &NativeAudioState::on_audio_ready, m_native);

    const aaudio_result_t open_res = AAudioStreamBuilder_openStream(builder, &m_native->stream);
    AAudioStreamBuilder_delete(builder);

    if (open_res != AAUDIO_OK || !m_native->stream) {
        close_hardware_stream();
        return false;
    }

    const i32 actual_rate = AAudioStream_getSampleRate(m_native->stream);
    if (actual_rate > 0) {
        m_sample_rate = static_cast<u32>(actual_rate);
    }

    if (AAudioStream_requestStart(m_native->stream) != AAUDIO_OK) {
        close_hardware_stream();
        return false;
    }
    return true;
#elif defined(_WIN32)
    if (waveInGetNumDevs() == 0) {
        close_hardware_stream();
        return false;
    }

    WAVEFORMATEX wfx{};
    wfx.wFormatTag = WAVE_FORMAT_PCM;
    wfx.nChannels = 1;
    wfx.nSamplesPerSec = m_sample_rate;
    wfx.wBitsPerSample = 16;
    wfx.nBlockAlign = static_cast<WORD>(wfx.nChannels * (wfx.wBitsPerSample / 8));
    wfx.nAvgBytesPerSec = wfx.nSamplesPerSec * wfx.nBlockAlign;

    if (waveInOpen(&m_native->wave_in, WAVE_MAPPER, &wfx, 0, 0, CALLBACK_NULL) != MMSYSERR_NOERROR ||
        !m_native->wave_in) {
        close_hardware_stream();
        return false;
    }

    for (usize i = 0; i < NativeAudioState::kNumBuffers; ++i) {
        auto& hdr = m_native->headers[i];
        hdr = WAVEHDR{};
        hdr.lpData = reinterpret_cast<LPSTR>(m_native->pcm_buffers[i]);
        hdr.dwBufferLength = static_cast<DWORD>(NativeAudioState::kBufferSamples * sizeof(i16));
        if (waveInPrepareHeader(m_native->wave_in, &hdr, sizeof(WAVEHDR)) == MMSYSERR_NOERROR) {
            m_native->headers_prepared[i] = true;
            waveInAddBuffer(m_native->wave_in, &hdr, sizeof(WAVEHDR));
        }
    }

    if (waveInStart(m_native->wave_in) != MMSYSERR_NOERROR) {
        close_hardware_stream();
        return false;
    }
    return true;
#else
    close_hardware_stream();
    return false;
#endif
}

void AudioCapture::close_hardware_stream() {
    if (!m_native) return;

#if defined(__ANDROID__)
    if (m_native->stream) {
        AAudioStream_requestStop(m_native->stream);
        AAudioStream_close(m_native->stream);
        m_native->stream = nullptr;
    }
#elif defined(_WIN32)
    if (m_native->wave_in) {
        waveInReset(m_native->wave_in);
        for (usize i = 0; i < NativeAudioState::kNumBuffers; ++i) {
            if (m_native->headers_prepared[i]) {
                waveInUnprepareHeader(m_native->wave_in, &m_native->headers[i], sizeof(WAVEHDR));
                m_native->headers_prepared[i] = false;
            }
        }
        waveInClose(m_native->wave_in);
        m_native->wave_in = nullptr;
    }
#endif

    delete m_native;
    m_native = nullptr;
}

bool AudioCapture::poll_hardware_stream() {
    if (!m_native || m_force_synthetic) return false;

#if defined(__ANDROID__)
    Array<f32> captured;
    {
        keel::LockGuard<keel::Mutex> lock(m_native->mutex);
        if (!m_native->staging.empty()) {
            captured = keel::move(m_native->staging);
            m_native->staging.clear();
        }
    }
    if (!captured.empty()) {
        push_hardware_samples(captured);
        return true;
    }
    return false;
#elif defined(_WIN32)
    if (!m_native->wave_in) return false;

    bool got_any = false;
    f32 conv_buf[NativeAudioState::kBufferSamples]{};
    constexpr f32 kScale = 1.0f / 32768.0f;

    for (usize i = 0; i < NativeAudioState::kNumBuffers; ++i) {
        auto& hdr = m_native->headers[i];
        if ((hdr.dwFlags & WHDR_DONE) != 0) {
            const usize samples_read = keel::min<usize>(
                hdr.dwBytesRecorded / sizeof(i16),
                NativeAudioState::kBufferSamples
            );
            if (samples_read > 0) {
                const i16* src = m_native->pcm_buffers[i];
                for (usize s = 0; s < samples_read; ++s) {
                    conv_buf[s] = static_cast<f32>(src[s]) * kScale;
                }
                push_hardware_samples(ArrayRef<f32>(conv_buf, samples_read));
                got_any = true;
            }
            hdr.dwBytesRecorded = 0;
            waveInAddBuffer(m_native->wave_in, &hdr, sizeof(WAVEHDR));
        }
    }
    return got_any;
#else
    return false;
#endif
}

bool AudioCapture::initialize(u32 sample_rate) {
    if (m_initialized) return true;

    m_sample_rate = (sample_rate > 0) ? keel::clamp<u32>(sample_rate, 8000u, 192000u) : kDefaultSampleRate;
    m_waveform.clear();
    m_waveform.resize(kWaveformWindowSamples, 0.0f);
    m_spectrum_db.clear();
    m_spectrum_db.resize(kSpectrumBins, -95.0f);

    if (!m_force_synthetic && open_hardware_stream()) {
        m_using_synthetic = false;
    } else {
        m_using_synthetic = true;
    }

    m_initialized = true;
    m_capturing = true;
    update_frame(0.016);
    return true;
}

void AudioCapture::shutdown() {
    close_hardware_stream();
    m_initialized = false;
    m_capturing = false;
}

bool AudioCapture::set_capturing(bool enabled) {
    m_capturing = enabled;
#if defined(__ANDROID__)
    if (m_native && m_native->stream) {
        if (enabled) {
            AAudioStream_requestStart(m_native->stream);
        } else {
            AAudioStream_requestPause(m_native->stream);
        }
    }
#elif defined(_WIN32)
    if (m_native && m_native->wave_in) {
        if (enabled) {
            waveInStart(m_native->wave_in);
        } else {
            waveInStop(m_native->wave_in);
        }
    }
#endif
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

    const f64 safe_dt = keel::is_finite(dt_seconds) ? keel::clamp(dt_seconds, 0.0, 0.25) : 0.0;
    m_sim_time_sec += safe_dt;

    if (!m_got_external_samples_this_frame) {
        poll_hardware_stream();
    }

    if (!m_got_external_samples_this_frame && (m_using_synthetic || m_force_synthetic)) {
        m_using_synthetic = true;
        if (m_waveform.size() < kWaveformWindowSamples) {
            m_waveform.resize(kWaveformWindowSamples, 0.0f);
        }
        const f64 desired = keel::clamp(keel::round(safe_dt * static_cast<f64>(m_sample_rate)), 128.0, 1024.0);
        const usize new_samples = keel::min<usize>(static_cast<usize>(desired), m_waveform.size());
        const usize keep = m_waveform.size() - new_samples;
        if (keep > 0) {
            mem_move(m_waveform.data(), m_waveform.data() + new_samples, keep * sizeof(f32));
        }

        const f64 two_pi = 2.0 * keel::kPi;
        const f64 raw_f0 = keel::is_finite(m_synth_freq_hz) ? static_cast<f64>(m_synth_freq_hz) : 1000.0;
        const f64 f0 = keel::clamp(raw_f0, 60.0, 18000.0);
        const f64 phase_step = two_pi * f0 / static_cast<f64>(m_sample_rate);
        const f32 safe_noise_db = keel::is_finite(m_synth_noise_db) ? keel::clamp(m_synth_noise_db, -120.0f, 0.0f) : -52.0f;
        const f32 noise_amp = keel::pow(10.0f, safe_noise_db / 20.0f);
        const f32 am_mod = 0.65f + 0.25f * keel::sin(static_cast<f32>(m_sim_time_sec * 2.2));

        u32 rng = static_cast<u32>(m_slice_sequence * 1664525u + 1013904223u);
        for (usize i = keep; i < m_waveform.size(); ++i) {
            rng = rng * 1664525u + 1013904223u;
            const f32 white = (static_cast<f32>(rng & 0xFFFFu) / 32768.0f) - 1.0f;

            const f32 s1 = 0.48f * am_mod * static_cast<f32>(keel::sin(m_synth_phase));
            const f32 s2 = 0.14f * static_cast<f32>(keel::sin(m_synth_phase * 2.0));
            const f32 s3 = 0.06f * static_cast<f32>(keel::sin(m_synth_phase * 3.0));
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
        const f32 raw_s = window_start[i];
        const f32 s = keel::is_finite(raw_s) ? raw_s : 0.0f;
        sum_sq += static_cast<f64>(s) * s;
        peak = keel::max(peak, keel::abs(s));

        const f32 w = 0.5f * (1.0f - keel::cos(2.0f * keel::as<f32>(keel::kPi) * static_cast<f32>(i) / static_cast<f32>(kFftSize - 1)));
        fft_buf[i] = {s * w, 0.0f};
    }

    const f32 rms = static_cast<f32>(keel::sqrt(sum_sq / static_cast<f64>(kFftSize)));
    m_rms_dbfs = keel::clamp(20.0f * keel::log10(keel::max(rms, 1e-5f)), -60.0f, 0.0f);
    m_peak_dbfs = keel::clamp(20.0f * keel::log10(keel::max(peak, 1e-5f)), -60.0f, 0.0f);
    m_stereo_meter_db[0] = m_rms_dbfs;
    m_stereo_meter_db[1] = keel::clamp(m_rms_dbfs - 1.2f, -60.0f, 0.0f);

    radix2_fft(fft_buf, kFftSize);

    const f32 inv_norm = 2.0f / static_cast<f32>(kFftSize);
    f32 max_bin_db = -120.0f;
    usize max_bin_idx = 1;

    for (usize k = 0; k < kSpectrumBins; ++k) {
        const f32 mag = fft_buf[k].abs() * inv_norm;
        const f32 db = keel::clamp(20.0f * keel::log10(keel::max(mag, 1e-6f)), -110.0f, 0.0f);
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
