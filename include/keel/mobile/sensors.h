#pragma once

#include "keel/array.h"
#include "keel/mobile/types.h"

namespace keel {

// Supported hardware & fused sensor modalities across Android NDK (ASensor) and iOS (CoreMotion).
enum class SensorType : u8 {
    Accelerometer = 0,  // 3-axis acceleration including gravity (m/s^2)
    LinearAcceleration, // 3-axis linear acceleration excluding gravity (m/s^2)
    Gravity,            // 3-axis gravity vector (m/s^2)
    Gyroscope,          // 3-axis angular velocity (rad/s)
    Magnetometer,       // 3-axis geomagnetic field (uT microtesla)
    RotationVector,     // Fused 9-DOF / 6-DOF attitude quaternion + Euler angles
    Barometer,          // Atmospheric pressure (hPa / mbar) & barometric altitude (m)
    AmbientLight,       // Ambient illuminance (lux)
    Proximity,          // Proximity distance (cm) & near/far flag
    StepCounter,        // Cumulative pedometer steps
    Count
};

// Reference frame for computing Pitch and Roll from device attitude.
enum class AttitudeReference : u8 {
    Handheld = 0, // 0 deg Pitch/Roll when phone is held upright in hand (screen facing user)
    FlatTable     // 0 deg Pitch/Roll when phone lies flat on a horizontal table (screen facing ceiling)
};

// Sampling rate presets matching Android NDK ASensorManager rates and iOS CMMotionManager intervals.
enum class SensorRate : u8 {
    Ui = 0,   // ~16.6 Hz (60,000 us) - Low battery impact for general UI orientation
    Normal,   // ~50 Hz   (20,000 us) - Standard telemetry & navigation
    Game,     // ~100 Hz  (10,000 us) - Responsive horizon, AR, and motion controls
    Fastest   // ~200 Hz+ ( 5,000 us) - High-rate DSP vibration & spectral analysis
};

// Metadata describing an available physical or fused sensor on the device.
struct SensorInfo {
    SensorType type = SensorType::Accelerometer;
    InplaceString<64> name{};
    InplaceString<64> vendor{};
    f32 max_range = 0.0f;
    f32 resolution = 0.0f;
    i32 min_delay_us = 0;
    bool available = false;
    bool enabled = false;
};

// Coherent frame-locked snapshot of all active mobile sensors.
struct SensorSnapshot {
    u64 timestamp_ns = 0;
    u64 sample_sequence = 0;

    // 3-Axis IMU & Magnetic
    Vec3f accelerometer{0.0f, 0.0f, 9.80665f};
    Vec3f linear_acceleration{0.0f, 0.0f, 0.0f};
    Vec3f gravity{0.0f, 0.0f, 9.80665f};
    Vec3f gyroscope{0.0f, 0.0f, 0.0f};
    Vec3f magnetometer{21.5f, 4.2f, -43.8f};

    // Fused Attitude & Flight Dynamics
    Quaternion attitude_quat{1.0f, 0.0f, 0.0f, 0.0f};
    EulerAngles attitude{};
    f32 g_load = 1.0f;
    f32 slip_skid = 0.0f;

    // Environmental & Biometric / Activity
    f32 pressure_hpa = 1013.25f;
    f32 sea_level_pressure_hpa = 1013.25f;
    f32 barometric_altitude_meters = 0.0f;
    f32 vertical_speed_mps = 0.0f;
    f32 ambient_light_lux = 320.0f;
    f32 proximity_cm = 5.0f;
    bool proximity_near = false;
    u64 step_count = 0;
};

// Simulator synthesis parameters for testing IMU, Barometer, and Environmental sensors on Desktop.
struct SensorSimConfig {
    bool animate_motion = true;
    f32 manual_pitch_deg = 6.5f;
    f32 manual_roll_deg = -12.0f;
    f32 manual_heading_deg = 42.0f;
    f32 vibration_freq_hz = 18.0f;
    f32 vibration_amp_g = 0.15f;
    f32 sim_pressure_hpa = 1008.4f;
    f32 sim_light_lux = 450.0f;
    bool sim_proximity_near = false;
};

// Cross-platform Sensor Manager (Android NDK ASensorManager + Desktop Physics Simulator).
class SensorManager {
public:
    static constexpr usize kDefaultHistorySamples = 1024;
    static constexpr usize kSensorTypeCount = static_cast<usize>(SensorType::Count);

    SensorManager();
    ~SensorManager();

    SensorManager(const SensorManager&) = delete;
    SensorManager& operator=(const SensorManager&) = delete;

    bool initialize();
    void shutdown();

    void set_paused(bool paused);
    [[nodiscard]] bool is_paused() const noexcept { return m_paused; }

    bool set_sensor_enabled(SensorType type, bool enabled);
    [[nodiscard]] bool is_sensor_enabled(SensorType type) const noexcept;
    [[nodiscard]] bool is_sensor_available(SensorType type) const noexcept;

    void set_sampling_rate(SensorRate rate);
    [[nodiscard]] SensorRate sampling_rate() const noexcept { return m_rate; }
    [[nodiscard]] f32 estimated_sample_rate_hz() const noexcept;

    void update_frame(f64 dt_seconds);

    [[nodiscard]] const SensorSnapshot& snapshot() const noexcept { return m_snapshot; }
    [[nodiscard]] ArrayRef<SensorInfo> sensor_infos() const noexcept {
        return ArrayRef<SensorInfo>(m_infos, kSensorTypeCount);
    }

    void set_sea_level_pressure_hpa(f32 qnh_hpa) noexcept { m_sea_level_pressure_hpa = qnh_hpa; }
    [[nodiscard]] f32 sea_level_pressure_hpa() const noexcept { return m_sea_level_pressure_hpa; }

    void set_attitude_reference(AttitudeReference ref) noexcept;
    [[nodiscard]] AttitudeReference attitude_reference() const noexcept { return m_attitude_ref; }

    void set_screen_orientation(ScreenOrientation orientation) noexcept { m_screen_orientation = orientation; }
    [[nodiscard]] ScreenOrientation screen_orientation() const noexcept { return m_screen_orientation; }

    void tare_attitude() noexcept;
    void reset_attitude_tare() noexcept;
    [[nodiscard]] bool has_attitude_tare() const noexcept { return m_has_attitude_tare; }

    [[nodiscard]] ArrayRef<f32> accel_history_x() const noexcept { return m_accel_hist_x; }
    [[nodiscard]] ArrayRef<f32> accel_history_y() const noexcept { return m_accel_hist_y; }
    [[nodiscard]] ArrayRef<f32> accel_history_z() const noexcept { return m_accel_hist_z; }
    [[nodiscard]] ArrayRef<f32> accel_magnitude_history() const noexcept { return m_accel_hist_mag; }
    [[nodiscard]] ArrayRef<f32> gyro_magnitude_history() const noexcept { return m_gyro_hist_mag; }
    [[nodiscard]] ArrayRef<f32> mag_magnitude_history() const noexcept { return m_mag_hist_mag; }
    [[nodiscard]] ArrayRef<f32> baro_altitude_history() const noexcept { return m_baro_alt_hist; }

    [[nodiscard]] SensorSimConfig& sim_config() noexcept { return m_sim_config; }
    [[nodiscard]] const SensorSimConfig& sim_config() const noexcept { return m_sim_config; }
    void set_use_simulator_override(bool force_sim) noexcept { m_force_simulator = force_sim; }
    [[nodiscard]] bool is_using_simulator() const noexcept;

    [[nodiscard]] static f32 pressure_to_altitude_meters(f32 pressure_hpa, f32 sea_level_hpa = 1013.25f) noexcept;
    [[nodiscard]] static EulerAngles quaternion_to_euler(
        Quaternion q,
        AttitudeReference ref = AttitudeReference::FlatTable
    ) noexcept;

private:
    void poll_android_sensors();
    void step_simulator(f64 dt_seconds);
    void push_history_sample(const SensorSnapshot& snap);

    SensorInfo m_infos[kSensorTypeCount]{};
    SensorSnapshot m_snapshot{};

    SensorRate m_rate = SensorRate::Game;
    AttitudeReference m_attitude_ref = AttitudeReference::Handheld;
    ScreenOrientation m_screen_orientation = ScreenOrientation::Portrait;
    f32 m_measured_sample_rate_hz = 0.0f;
    bool m_has_attitude_tare = false;
    f32 m_pitch_tare_deg = 0.0f;
    f32 m_roll_tare_deg = 0.0f;
    f32 m_raw_untared_pitch_deg = 0.0f;
    f32 m_raw_untared_roll_deg = 0.0f;
    f32 m_sea_level_pressure_hpa = 1013.25f;
    f64 m_sim_time_sec = 0.0;
    bool m_initialized = false;
    bool m_paused = false;
    bool m_has_hardware_backend = false;
    bool m_force_simulator = false;
    SensorSimConfig m_sim_config{};

    Array<f32> m_accel_hist_x{};
    Array<f32> m_accel_hist_y{};
    Array<f32> m_accel_hist_z{};
    Array<f32> m_accel_hist_mag{};
    Array<f32> m_gyro_hist_mag{};
    Array<f32> m_mag_hist_mag{};
    Array<f32> m_baro_alt_hist{};

    struct AndroidImpl;
    AndroidImpl* m_android = nullptr;
};

} // namespace keel
