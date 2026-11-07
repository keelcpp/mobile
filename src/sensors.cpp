#include "keel/mobile/sensors.h"
#include "keel/algo.h"

#include <chrono>
#include <cmath>
#include <numbers>

#ifdef __ANDROID__
#include <android/looper.h>
#include <android/sensor.h>
#endif

namespace keel {

f32 Vec3f::magnitude() const noexcept {
    return std::sqrt(x * x + y * y + z * z);
}

namespace {

constexpr f32 kGravityStandard = 9.80665f;
constexpr f32 kRadToDeg = 180.0f / std::numbers::pi_v<f32>;
constexpr f32 kDegToRad = std::numbers::pi_v<f32> / 180.0f;

[[nodiscard]] constexpr i32 rate_to_delay_us(SensorRate rate) noexcept {
    switch (rate) {
        case SensorRate::Ui:      return 60000; // ~16.6 Hz
        case SensorRate::Normal:  return 20000; // 50 Hz
        case SensorRate::Game:    return 10000; // 100 Hz
        case SensorRate::Fastest: return 5000;  // 200 Hz
    }
    return 10000;
}

[[nodiscard]] Quaternion euler_to_quaternion(f32 pitch_deg, f32 roll_deg, f32 yaw_deg) noexcept {
    const f32 half_pitch = (pitch_deg * kDegToRad) * 0.5f;
    const f32 half_roll  = (roll_deg  * kDegToRad) * 0.5f;
    const f32 half_yaw   = (yaw_deg   * kDegToRad) * 0.5f;

    const f32 cx = std::cos(half_pitch);
    const f32 sx = std::sin(half_pitch);
    const f32 cy = std::cos(half_roll);
    const f32 sy = std::sin(half_roll);
    const f32 cz = std::cos(half_yaw);
    const f32 sz = std::sin(half_yaw);

    return Quaternion{
        cx * cy * cz + sx * sy * sz,
        sx * cy * cz - cx * sy * sz,
        cx * sy * cz + sx * cy * sz,
        cx * cy * sz - sx * sy * cz
    };
}

void append_ring_sample(Array<f32>& buf, f32 value, usize max_samples) {
    if (buf.size() < max_samples) {
        buf.push_back(value);
    } else if (!buf.empty()) {
        mem_move(buf.data(), buf.data() + 1, (buf.size() - 1) * sizeof(f32));
        buf.back() = value;
    }
}

#ifdef __ANDROID__
constexpr int kAndroidTypeAccelerometer      = ASENSOR_TYPE_ACCELEROMETER;       // 1
constexpr int kAndroidTypeMagneticField      = ASENSOR_TYPE_MAGNETIC_FIELD;      // 2
constexpr int kAndroidTypeGyroscope          = ASENSOR_TYPE_GYROSCOPE;           // 4
constexpr int kAndroidTypeLight              = ASENSOR_TYPE_LIGHT;               // 5
constexpr int kAndroidTypePressure           = 6;  // TYPE_PRESSURE
constexpr int kAndroidTypeProximity          = ASENSOR_TYPE_PROXIMITY;           // 8
constexpr int kAndroidTypeGravity            = 9;  // TYPE_GRAVITY
constexpr int kAndroidTypeLinearAcceleration = 10; // TYPE_LINEAR_ACCELERATION
constexpr int kAndroidTypeRotationVector     = 11; // TYPE_ROTATION_VECTOR
constexpr int kAndroidTypeStepCounter        = 19; // TYPE_STEP_COUNTER

[[nodiscard]] int to_android_sensor_type(SensorType type) noexcept {
    switch (type) {
        case SensorType::Accelerometer:      return kAndroidTypeAccelerometer;
        case SensorType::LinearAcceleration: return kAndroidTypeLinearAcceleration;
        case SensorType::Gravity:            return kAndroidTypeGravity;
        case SensorType::Gyroscope:          return kAndroidTypeGyroscope;
        case SensorType::Magnetometer:       return kAndroidTypeMagneticField;
        case SensorType::RotationVector:     return kAndroidTypeRotationVector;
        case SensorType::Barometer:          return kAndroidTypePressure;
        case SensorType::AmbientLight:       return kAndroidTypeLight;
        case SensorType::Proximity:          return kAndroidTypeProximity;
        case SensorType::StepCounter:        return kAndroidTypeStepCounter;
        case SensorType::Count:              break;
    }
    return -1;
}
#endif

} // namespace

struct SensorManager::AndroidImpl {
#ifdef __ANDROID__
    ASensorManager* manager = nullptr;
    ALooper* looper = nullptr;
    ASensorEventQueue* queue = nullptr;
    const ASensor* sensors[kSensorTypeCount]{};
    Vec3f filtered_accel{0.0f, kGravityStandard, 0.0f};
    bool has_filtered_accel = false;
    i64 last_accel_timestamp_ns = 0;
    i64 last_baro_timestamp_ns = 0;
    f32 prev_baro_altitude_meters = 0.0f;
    bool has_prev_baro_altitude = false;
    u64 initial_step_offset = 0;
    bool has_initial_steps = false;
#endif
};

SensorManager::SensorManager()
    : m_android(new AndroidImpl{})
{
    m_accel_hist_x.reserve(kDefaultHistorySamples);
    m_accel_hist_y.reserve(kDefaultHistorySamples);
    m_accel_hist_z.reserve(kDefaultHistorySamples);
    m_accel_hist_mag.reserve(kDefaultHistorySamples);
    m_gyro_hist_mag.reserve(kDefaultHistorySamples);
    m_mag_hist_mag.reserve(kDefaultHistorySamples);
    m_baro_alt_hist.reserve(kDefaultHistorySamples);
}

SensorManager::~SensorManager() {
    shutdown();
    delete m_android;
    m_android = nullptr;
}

bool SensorManager::initialize() {
    if (m_initialized) return true;

    const auto init_default_info = [&](SensorType t, const char* name, f32 max_range, f32 res) {
        auto& info = m_infos[static_cast<usize>(t)];
        info.type = t;
        info.name.set_static(name);
        info.vendor.set_static("keel Simulated IMU");
        info.max_range = max_range;
        info.resolution = res;
        info.min_delay_us = 5000;
        info.available = true;
        info.enabled = true;
    };

    init_default_info(SensorType::Accelerometer,      "3-Axis Accelerometer",       156.9f, 0.0024f);
    init_default_info(SensorType::LinearAcceleration, "Linear Acceleration",        156.9f, 0.0024f);
    init_default_info(SensorType::Gravity,            "Gravity Vector",               9.81f, 0.001f);
    init_default_info(SensorType::Gyroscope,          "3-Axis MEMS Gyroscope",       34.9f, 0.001f);
    init_default_info(SensorType::Magnetometer,       "3-Axis Geomagnetic Sensor", 4900.0f, 0.15f);
    init_default_info(SensorType::RotationVector,     "9-DOF Attitude Fusion",        1.0f, 0.0001f);
    init_default_info(SensorType::Barometer,          "MEMS Barometric Pressure",  1260.0f, 0.01f);
    init_default_info(SensorType::AmbientLight,       "Ambient Light Photodiode", 65535.0f, 1.0f);
    init_default_info(SensorType::Proximity,          "IR Proximity Sensor",          5.0f, 1.0f);
    init_default_info(SensorType::StepCounter,        "Hardware Pedometer",       100000.0f, 1.0f);

#ifdef __ANDROID__
#if __ANDROID_API__ >= 26
    m_android->manager = ASensorManager_getInstanceForPackage("io.keelcpp.mobile");
#else
    m_android->manager = ASensorManager_getInstance();
#endif
    if (m_android->manager) {
        m_android->looper = ALooper_forThread();
        if (!m_android->looper) {
            m_android->looper = ALooper_prepare(ALOOPER_PREPARE_ALLOW_NON_CALLBACKS);
        }
        m_android->queue = ASensorManager_createEventQueue(
            m_android->manager,
            m_android->looper,
            ALOOPER_POLL_CALLBACK,
            nullptr,
            nullptr
        );

        if (m_android->queue) {
            m_has_hardware_backend = true;
            const i32 delay_us = rate_to_delay_us(m_rate);

            for (usize i = 0; i < kSensorTypeCount; ++i) {
                const auto st = static_cast<SensorType>(i);
                const int android_type = to_android_sensor_type(st);
                const ASensor* s = ASensorManager_getDefaultSensor(m_android->manager, android_type);
                m_android->sensors[i] = s;

                auto& info = m_infos[i];
                if (s) {
                    if (const char* sname = ASensor_getName(s)) {
                        info.name.set(StringRef(sname));
                    }
                    if (const char* svend = ASensor_getVendor(s)) {
                        info.vendor.set(StringRef(svend));
                    } else {
                        info.vendor.set_static("Android NDK");
                    }
                    info.resolution = ASensor_getResolution(s);
                    info.min_delay_us = ASensor_getMinDelay(s);
                    info.available = true;
                    if (ASensorEventQueue_enableSensor(m_android->queue, s) >= 0) {
                        const i32 target_us = keel::max(delay_us, info.min_delay_us);
                        ASensorEventQueue_setEventRate(m_android->queue, s, target_us);
                        info.enabled = true;
                    }
                } else {
                    info.available = false;
                    info.enabled = false;
                }
            }
        }
    }
#endif

    for (usize i = 0; i < kDefaultHistorySamples / 2; ++i) {
        step_simulator(0.01);
        push_history_sample(m_snapshot);
    }

    m_initialized = true;
    return true;
}

void SensorManager::shutdown() {
    if (!m_initialized) return;

#ifdef __ANDROID__
    if (m_android && m_android->manager && m_android->queue) {
        for (usize i = 0; i < kSensorTypeCount; ++i) {
            if (m_android->sensors[i] && m_infos[i].enabled) {
                ASensorEventQueue_disableSensor(m_android->queue, m_android->sensors[i]);
            }
            m_android->sensors[i] = nullptr;
        }
        ASensorManager_destroyEventQueue(m_android->manager, m_android->queue);
        m_android->queue = nullptr;
    }
#endif

    m_initialized = false;
}

void SensorManager::set_paused(bool paused) {
    if (m_paused == paused) return;
    m_paused = paused;

#ifdef __ANDROID__
    if (m_android && m_android->queue) {
        const i32 delay_us = rate_to_delay_us(m_rate);
        for (usize i = 0; i < kSensorTypeCount; ++i) {
            const ASensor* s = m_android->sensors[i];
            if (!s || !m_infos[i].enabled) continue;
            if (paused) {
                ASensorEventQueue_disableSensor(m_android->queue, s);
            } else {
                ASensorEventQueue_enableSensor(m_android->queue, s);
                ASensorEventQueue_setEventRate(m_android->queue, s, keel::max(delay_us, m_infos[i].min_delay_us));
            }
        }
    }
#endif
}

bool SensorManager::set_sensor_enabled(SensorType type, bool enabled) {
    const auto idx = static_cast<usize>(type);
    if (idx >= kSensorTypeCount) return false;

    auto& info = m_infos[idx];
    if (!info.available && !is_using_simulator()) return false;
    info.enabled = enabled;

#ifdef __ANDROID__
    if (m_android && m_android->queue && m_android->sensors[idx] && !m_paused) {
        if (enabled) {
            ASensorEventQueue_enableSensor(m_android->queue, m_android->sensors[idx]);
            const i32 delay_us = keel::max(rate_to_delay_us(m_rate), info.min_delay_us);
            ASensorEventQueue_setEventRate(m_android->queue, m_android->sensors[idx], delay_us);
        } else {
            ASensorEventQueue_disableSensor(m_android->queue, m_android->sensors[idx]);
        }
    }
#endif
    return true;
}

bool SensorManager::is_sensor_enabled(SensorType type) const noexcept {
    const auto idx = static_cast<usize>(type);
    return idx < kSensorTypeCount ? m_infos[idx].enabled : false;
}

bool SensorManager::is_sensor_available(SensorType type) const noexcept {
    const auto idx = static_cast<usize>(type);
    return idx < kSensorTypeCount ? (m_infos[idx].available || is_using_simulator()) : false;
}

void SensorManager::set_sampling_rate(SensorRate rate) {
    m_rate = rate;
#ifdef __ANDROID__
    if (m_android && m_android->queue && !m_paused) {
        const i32 delay_us = rate_to_delay_us(m_rate);
        for (usize i = 0; i < kSensorTypeCount; ++i) {
            if (m_android->sensors[i] && m_infos[i].enabled) {
                ASensorEventQueue_setEventRate(
                    m_android->queue,
                    m_android->sensors[i],
                    keel::max(delay_us, m_infos[i].min_delay_us)
                );
            }
        }
    }
#endif
}

f32 SensorManager::estimated_sample_rate_hz() const noexcept {
    if (!is_using_simulator() && m_measured_sample_rate_hz >= 5.0f) {
        return m_measured_sample_rate_hz;
    }
    switch (m_rate) {
        case SensorRate::Ui:      return 16.6f;
        case SensorRate::Normal:  return 50.0f;
        case SensorRate::Game:    return 100.0f;
        case SensorRate::Fastest: return 200.0f;
    }
    return 100.0f;
}

bool SensorManager::is_using_simulator() const noexcept {
    return m_force_simulator || !m_has_hardware_backend;
}

void SensorManager::update_frame(f64 dt_seconds) {
    if (m_paused) return;

    const f64 clamped_dt = keel::clamp(dt_seconds, 0.001, 0.1);
    if (is_using_simulator()) {
        step_simulator(clamped_dt);
        push_history_sample(m_snapshot);
    } else {
        poll_android_sensors();
    }
}

void SensorManager::poll_android_sensors() {
#ifdef __ANDROID__
    if (!m_android || !m_android->queue) return;

    ASensorEvent ev{};
    bool got_event = false;
    bool pushed_accel_history = false;

    const ScreenOrientation orient = m_screen_orientation;
    const auto remap_to_screen = [orient](f32 x, f32 y, f32 z) noexcept -> Vec3f {
        switch (orient) {
            case ScreenOrientation::LandscapeLeft:      return {-y,  x, z};
            case ScreenOrientation::LandscapeRight:     return { y, -x, z};
            case ScreenOrientation::PortraitUpsideDown: return {-x, -y, z};
            default:                                    return { x,  y, z};
        }
    };

    const bool has_accelerometer = m_infos[static_cast<usize>(SensorType::Accelerometer)].available;

    while (ASensorEventQueue_getEvents(m_android->queue, &ev, 1) > 0) {
        got_event = true;
        m_snapshot.timestamp_ns = static_cast<u64>(ev.timestamp);

        switch (ev.type) {
            case kAndroidTypeAccelerometer: {
                m_snapshot.accelerometer = remap_to_screen(ev.acceleration.x, ev.acceleration.y, ev.acceleration.z);
                m_snapshot.g_load = m_snapshot.accelerometer.magnitude() / kGravityStandard;
                m_snapshot.slip_skid = keel::clamp(-m_snapshot.accelerometer.x / kGravityStandard, -1.0f, 1.0f);

                f32 dt_accel = 0.01f;
                if (m_android->last_accel_timestamp_ns > 0 && ev.timestamp > m_android->last_accel_timestamp_ns) {
                    const f32 raw_dt = static_cast<f32>(ev.timestamp - m_android->last_accel_timestamp_ns) * 1e-9f;
                    if (raw_dt >= 0.0005f && raw_dt <= 0.25f) {
                        dt_accel = raw_dt;
                        const f32 inst_rate_hz = keel::clamp(1.0f / raw_dt, 5.0f, 500.0f);
                        m_measured_sample_rate_hz = (m_measured_sample_rate_hz <= 0.0f)
                            ? inst_rate_hz
                            : (m_measured_sample_rate_hz + 0.05f * (inst_rate_hz - m_measured_sample_rate_hz));
                    }
                }
                m_android->last_accel_timestamp_ns = ev.timestamp;

                constexpr f32 kFilterTauSec = 0.035f;
                const f32 alpha = dt_accel / (kFilterTauSec + dt_accel);
                if (!m_android->has_filtered_accel) {
                    m_android->filtered_accel = m_snapshot.accelerometer;
                    m_android->has_filtered_accel = true;
                } else {
                    m_android->filtered_accel.x += alpha * (m_snapshot.accelerometer.x - m_android->filtered_accel.x);
                    m_android->filtered_accel.y += alpha * (m_snapshot.accelerometer.y - m_android->filtered_accel.y);
                    m_android->filtered_accel.z += alpha * (m_snapshot.accelerometer.z - m_android->filtered_accel.z);
                }

                const Vec3f& g = m_android->filtered_accel;
                const f32 xy_mag = std::sqrt(g.x * g.x + g.y * g.y);
                const f32 yz_mag = std::sqrt(g.y * g.y + g.z * g.z);
                if (m_attitude_ref == AttitudeReference::Handheld) {
                    m_raw_untared_pitch_deg = keel::clamp(std::atan2(-g.z, xy_mag) * kRadToDeg, -90.0f, 90.0f);
                } else {
                    m_raw_untared_pitch_deg = keel::clamp(std::atan2(g.y, g.z) * kRadToDeg, -90.0f, 90.0f);
                }
                m_raw_untared_roll_deg = std::atan2(-g.x, yz_mag) * kRadToDeg;
                m_snapshot.attitude.pitch_deg = keel::clamp(
                    m_raw_untared_pitch_deg - (m_has_attitude_tare ? m_pitch_tare_deg : 0.0f),
                    -90.0f,
                    90.0f
                );
                m_snapshot.attitude.roll_deg = m_raw_untared_roll_deg - (m_has_attitude_tare ? m_roll_tare_deg : 0.0f);
                m_snapshot.attitude_quat = euler_to_quaternion(
                    m_snapshot.attitude.pitch_deg,
                    m_snapshot.attitude.roll_deg,
                    m_snapshot.attitude.yaw_deg
                );

                push_history_sample(m_snapshot);
                pushed_accel_history = true;
                break;
            }

            case kAndroidTypeLinearAcceleration:
                m_snapshot.linear_acceleration = remap_to_screen(ev.acceleration.x, ev.acceleration.y, ev.acceleration.z);
                break;

            case kAndroidTypeGravity:
                m_snapshot.gravity = remap_to_screen(ev.vector.x, ev.vector.y, ev.vector.z);
                break;

            case kAndroidTypeGyroscope:
                m_snapshot.gyroscope = remap_to_screen(ev.vector.x, ev.vector.y, ev.vector.z);
                break;

            case kAndroidTypeMagneticField:
                m_snapshot.magnetometer = remap_to_screen(ev.magnetic.x, ev.magnetic.y, ev.magnetic.z);
                break;

            case kAndroidTypeRotationVector: {
                const Vec3f qv = remap_to_screen(ev.data[0], ev.data[1], ev.data[2]);
                const f32 qw = (ev.data[3] != 0.0f)
                    ? ev.data[3]
                    : std::sqrt(keel::max(0.0f, 1.0f - (qv.x * qv.x + qv.y * qv.y + qv.z * qv.z)));
                EulerAngles raw_euler = quaternion_to_euler({qw, qv.x, qv.y, qv.z}, m_attitude_ref);
                m_snapshot.attitude.yaw_deg = raw_euler.yaw_deg;
                m_snapshot.attitude.heading_deg = raw_euler.heading_deg;
                if (!has_accelerometer) {
                    m_snapshot.attitude_quat = {qw, qv.x, qv.y, qv.z};
                    m_raw_untared_pitch_deg = raw_euler.pitch_deg;
                    m_raw_untared_roll_deg  = raw_euler.roll_deg;
                    m_snapshot.attitude.pitch_deg = keel::clamp(
                        raw_euler.pitch_deg - (m_has_attitude_tare ? m_pitch_tare_deg : 0.0f),
                        -90.0f,
                        90.0f
                    );
                    m_snapshot.attitude.roll_deg = raw_euler.roll_deg - (m_has_attitude_tare ? m_roll_tare_deg : 0.0f);
                }
                break;
            }

            case kAndroidTypePressure: {
                const f32 new_alt = pressure_to_altitude_meters(ev.pressure, m_sea_level_pressure_hpa);
                if (m_android->has_prev_baro_altitude && m_android->last_baro_timestamp_ns > 0 &&
                    ev.timestamp > m_android->last_baro_timestamp_ns) {
                    const f32 dt_baro = static_cast<f32>(ev.timestamp - m_android->last_baro_timestamp_ns) * 1e-9f;
                    if (dt_baro >= 0.005f && dt_baro <= 2.0f) {
                        const f32 raw_vs = (new_alt - m_android->prev_baro_altitude_meters) / dt_baro;
                        m_snapshot.vertical_speed_mps += 0.25f * (raw_vs - m_snapshot.vertical_speed_mps);
                    }
                }
                m_android->prev_baro_altitude_meters = new_alt;
                m_android->last_baro_timestamp_ns = ev.timestamp;
                m_android->has_prev_baro_altitude = true;
                m_snapshot.pressure_hpa = ev.pressure;
                m_snapshot.sea_level_pressure_hpa = m_sea_level_pressure_hpa;
                m_snapshot.barometric_altitude_meters = new_alt;
                break;
            }

            case kAndroidTypeLight:
                m_snapshot.ambient_light_lux = ev.light;
                break;

            case kAndroidTypeProximity:
                m_snapshot.proximity_cm = ev.distance;
                m_snapshot.proximity_near = (ev.distance < m_infos[static_cast<usize>(SensorType::Proximity)].max_range);
                break;

            case kAndroidTypeStepCounter: {
                const auto raw_steps = static_cast<u64>(ev.u64.step_counter);
                if (!m_android->has_initial_steps) {
                    m_android->initial_step_offset = raw_steps;
                    m_android->has_initial_steps = true;
                }
                m_snapshot.step_count = (raw_steps >= m_android->initial_step_offset)
                    ? (raw_steps - m_android->initial_step_offset)
                    : raw_steps;
                break;
            }
            default:
                break;
        }
    }

    if (got_event) {
        ++m_snapshot.sample_sequence;
        if (!pushed_accel_history) {
            push_history_sample(m_snapshot);
        }
    }
#endif
}

void SensorManager::step_simulator(f64 dt_seconds) {
    m_sim_time_sec += dt_seconds;
    const f32 t = static_cast<f32>(m_sim_time_sec);

    f32 pitch_deg = m_sim_config.manual_pitch_deg;
    f32 roll_deg  = m_sim_config.manual_roll_deg;
    f32 yaw_deg   = m_sim_config.manual_heading_deg;

    if (m_sim_config.animate_motion) {
        pitch_deg += 8.5f * std::sin(t * 0.85f) + 2.0f * std::cos(t * 2.1f);
        roll_deg  += 18.0f * std::sin(t * 0.55f) + 3.5f * std::sin(t * 1.4f);
        yaw_deg    = std::fmod(m_sim_config.manual_heading_deg + t * 6.0f + 360.0f, 360.0f);
    }

    const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()
    ).count();
    m_snapshot.timestamp_ns = static_cast<u64>(keel::max<i64>(0, now_ns));
    ++m_snapshot.sample_sequence;

    m_snapshot.attitude_quat = euler_to_quaternion(pitch_deg, roll_deg, yaw_deg);
    m_raw_untared_pitch_deg = pitch_deg;
    m_raw_untared_roll_deg  = roll_deg;
    m_snapshot.attitude.pitch_deg   = keel::clamp(pitch_deg - (m_has_attitude_tare ? m_pitch_tare_deg : 0.0f), -90.0f, 90.0f);
    m_snapshot.attitude.roll_deg    = roll_deg - (m_has_attitude_tare ? m_roll_tare_deg : 0.0f);
    m_snapshot.attitude.yaw_deg     = yaw_deg;
    m_snapshot.attitude.heading_deg = std::fmod(yaw_deg + 360.0f, 360.0f);

    const f32 pitch_rad = pitch_deg * kDegToRad;
    const f32 roll_rad  = roll_deg  * kDegToRad;
    const Vec3f grav{
        kGravityStandard * std::sin(roll_rad) * std::cos(pitch_rad),
        -kGravityStandard * std::sin(pitch_rad),
        kGravityStandard * std::cos(roll_rad) * std::cos(pitch_rad)
    };
    m_snapshot.gravity = grav;

    const f32 vib_omega = 2.0f * std::numbers::pi_v<f32> * m_sim_config.vibration_freq_hz;
    const f32 vib_amp_ms2 = m_sim_config.vibration_amp_g * kGravityStandard;
    const Vec3f lin_acc{
        vib_amp_ms2 * std::sin(vib_omega * t) + 0.35f * vib_amp_ms2 * std::sin(2.4f * vib_omega * t),
        vib_amp_ms2 * std::cos(vib_omega * t * 0.9f),
        vib_amp_ms2 * 0.7f * std::sin(vib_omega * t * 1.3f)
    };
    m_snapshot.linear_acceleration = lin_acc;
    m_snapshot.accelerometer = {
        grav.x + lin_acc.x,
        grav.y + lin_acc.y,
        grav.z + lin_acc.z
    };
    m_snapshot.g_load = m_snapshot.accelerometer.magnitude() / kGravityStandard;
    m_snapshot.slip_skid = keel::clamp(std::sin(roll_rad) * 0.35f, -1.0f, 1.0f);

    if (m_sim_config.animate_motion) {
        m_snapshot.gyroscope = {
            (8.5f * 0.85f * kDegToRad) * std::cos(t * 0.85f),
            (18.0f * 0.55f * kDegToRad) * std::cos(t * 0.55f),
            6.0f * kDegToRad
        };
    } else {
        m_snapshot.gyroscope = {0.0f, 0.0f, 0.0f};
    }

    const f32 head_rad = m_snapshot.attitude.heading_deg * kDegToRad;
    m_snapshot.magnetometer = {
        22.0f * std::cos(head_rad) + 0.6f * std::sin(vib_omega * t),
        22.0f * std::sin(head_rad) + 0.6f * std::cos(vib_omega * t),
        -42.5f + 1.2f * std::sin(pitch_rad)
    };

    const f32 prev_alt = m_snapshot.barometric_altitude_meters;
    const f32 pressure_wave = m_sim_config.animate_motion ? (0.45f * std::sin(t * 0.25f)) : 0.0f;
    m_snapshot.pressure_hpa = m_sim_config.sim_pressure_hpa + pressure_wave;
    m_snapshot.sea_level_pressure_hpa = m_sea_level_pressure_hpa;
    m_snapshot.barometric_altitude_meters = pressure_to_altitude_meters(m_snapshot.pressure_hpa, m_sea_level_pressure_hpa);
    m_snapshot.vertical_speed_mps = (dt_seconds > 0.0001)
        ? static_cast<f32>((m_snapshot.barometric_altitude_meters - prev_alt) / dt_seconds)
        : 0.0f;

    m_snapshot.ambient_light_lux = keel::max(1.0f, m_sim_config.sim_light_lux + 25.0f * std::sin(t * 0.4f));
    m_snapshot.proximity_near = m_sim_config.sim_proximity_near;
    m_snapshot.proximity_cm = m_sim_config.sim_proximity_near ? 0.0f : 5.0f;
    if (m_sim_config.animate_motion) {
        m_snapshot.step_count = static_cast<u64>(t * 1.6f);
    }
}

void SensorManager::push_history_sample(const SensorSnapshot& snap) {
    append_ring_sample(m_accel_hist_x, snap.accelerometer.x, kDefaultHistorySamples);
    append_ring_sample(m_accel_hist_y, snap.accelerometer.y, kDefaultHistorySamples);
    append_ring_sample(m_accel_hist_z, snap.accelerometer.z, kDefaultHistorySamples);
    const f32 norm_g = keel::clamp((snap.accelerometer.magnitude() - kGravityStandard) / (2.0f * kGravityStandard), -1.0f, 1.0f);
    append_ring_sample(m_accel_hist_mag, norm_g, kDefaultHistorySamples);
    append_ring_sample(m_gyro_hist_mag, keel::clamp(snap.gyroscope.magnitude() / 4.0f, -1.0f, 1.0f), kDefaultHistorySamples);
    append_ring_sample(m_mag_hist_mag, snap.magnetometer.magnitude(), kDefaultHistorySamples);
    append_ring_sample(m_baro_alt_hist, snap.barometric_altitude_meters, kDefaultHistorySamples);
}

void SensorManager::set_attitude_reference(AttitudeReference ref) noexcept {
    if (m_attitude_ref != ref) {
        m_attitude_ref = ref;
        reset_attitude_tare();
    }
}

void SensorManager::tare_attitude() noexcept {
    m_pitch_tare_deg = m_raw_untared_pitch_deg;
    m_roll_tare_deg  = m_raw_untared_roll_deg;
    m_has_attitude_tare = true;
    m_snapshot.attitude.pitch_deg = 0.0f;
    m_snapshot.attitude.roll_deg  = 0.0f;
}

void SensorManager::reset_attitude_tare() noexcept {
    m_has_attitude_tare = false;
    m_pitch_tare_deg = 0.0f;
    m_roll_tare_deg  = 0.0f;
}

f32 SensorManager::pressure_to_altitude_meters(f32 pressure_hpa, f32 sea_level_hpa) noexcept {
    if (pressure_hpa <= 0.0f || sea_level_hpa <= 0.0f) return 0.0f;
    return 44330.0f * (1.0f - std::pow(pressure_hpa / sea_level_hpa, 0.190294957f));
}

EulerAngles SensorManager::quaternion_to_euler(Quaternion q, AttitudeReference ref) noexcept {
    EulerAngles out{};

    const f32 ux = 2.0f * (q.x * q.z - q.w * q.y);
    const f32 uy = 2.0f * (q.y * q.z + q.w * q.x);
    const f32 uz = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);

    const f32 xy_mag = std::sqrt(ux * ux + uy * uy);
    const f32 yz_mag = std::sqrt(uy * uy + uz * uz);

    if (ref == AttitudeReference::Handheld) {
        out.pitch_deg = keel::clamp(std::atan2(-uz, xy_mag) * kRadToDeg, -90.0f, 90.0f);
    } else {
        out.pitch_deg = keel::clamp(std::atan2(uy, uz) * kRadToDeg, -90.0f, 90.0f);
    }

    out.roll_deg = std::atan2(-ux, yz_mag) * kRadToDeg;

    const f32 x_east  = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    const f32 x_north = 2.0f * (q.x * q.y + q.w * q.z);
    out.yaw_deg = std::atan2(-x_north, x_east) * kRadToDeg;
    out.heading_deg = std::fmod(out.yaw_deg + 360.0f, 360.0f);

    return out;
}

} // namespace keel
