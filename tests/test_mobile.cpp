#include "keel/mobile.h"
#include "keel/format.h"

#include <cmath>
#include <cstdlib>

using namespace keel;

namespace {

int g_failures = 0;

void check_impl(bool cond, const char* expr, const char* file, int line) {
    if (!cond) {
        keel::eprintln("[FAIL] %s:%d: %s", file, line, expr);
        ++g_failures;
    }
}

#define CHECK(expr) check_impl((expr), #expr, __FILE__, __LINE__)
#define CHECK_NEAR(a, b, tol) check_impl(std::abs(static_cast<double>(a) - static_cast<double>(b)) <= static_cast<double>(tol), #a " ~= " #b, __FILE__, __LINE__)

void test_sensors() {
    // Sea level standard pressure = 1013.25 hPa -> 0.0 m
    const f32 alt_sea = SensorManager::pressure_to_altitude_meters(1013.25f, 1013.25f);
    CHECK_NEAR(alt_sea, 0.0f, 0.05f);

    // ~900 hPa is roughly ~988 meters above sea level
    const f32 alt_900 = SensorManager::pressure_to_altitude_meters(900.0f, 1013.25f);
    CHECK(alt_900 > 950.0f && alt_900 < 1030.0f);

    // Identity quaternion (1, 0, 0, 0): device flat on table facing North
    const Quaternion identity{1.0f, 0.0f, 0.0f, 0.0f};
    const EulerAngles flat = SensorManager::quaternion_to_euler(identity, AttitudeReference::FlatTable);
    CHECK_NEAR(flat.pitch_deg, 0.0f, 0.01f);
    CHECK_NEAR(flat.roll_deg, 0.0f, 0.01f);
    CHECK_NEAR(flat.yaw_deg, 0.0f, 0.01f);

    // Rotation of +90 degrees around X axis (phone held upright in portrait mode)
    const f32 half_rad = 45.0f * (3.1415926535f / 180.0f);
    const Quaternion upright{std::cos(half_rad), std::sin(half_rad), 0.0f, 0.0f};
    const EulerAngles handheld = SensorManager::quaternion_to_euler(upright, AttitudeReference::Handheld);
    CHECK_NEAR(handheld.pitch_deg, 0.0f, 0.2f);
    CHECK_NEAR(handheld.roll_deg, 0.0f, 0.2f);

    SensorManager sensors;
    CHECK(sensors.initialize());
    sensors.set_sampling_rate(SensorRate::Game);
    sensors.set_use_simulator_override(true);
    sensors.sim_config().animate_motion = true;

    for (int i = 0; i < 15; ++i) {
        sensors.update_frame(1.0 / 60.0);
    }

    CHECK(sensors.accel_history_x().size() == SensorManager::kDefaultHistorySamples / 2 + 15u);
    CHECK(sensors.gyro_magnitude_history().size() == SensorManager::kDefaultHistorySamples / 2 + 15u);
    CHECK(sensors.baro_altitude_history().size() == SensorManager::kDefaultHistorySamples / 2 + 15u);
    CHECK(sensors.snapshot().accelerometer.magnitude() > 8.0f);

    // Tare should zero pitch and roll at current orientation
    sensors.sim_config().animate_motion = false;
    sensors.update_frame(0.0);
    sensors.tare_attitude();
    CHECK(sensors.has_attitude_tare());
    CHECK_NEAR(sensors.snapshot().attitude.pitch_deg, 0.0f, 0.25f);
    CHECK_NEAR(sensors.snapshot().attitude.roll_deg, 0.0f, 0.25f);

    sensors.reset_attitude_tare();
    CHECK(!sensors.has_attitude_tare());
    sensors.shutdown();
}

void test_camera() {
    CameraService cam;
    CHECK(cam.initialize());
    CHECK(cam.is_streaming());
    CHECK(cam.devices().size() >= 1u);

    cam.update_frame(1.0 / 60.0);

    const u32 w = cam.frame_width();
    const u32 h = cam.frame_height();
    CHECK(w > 0u && h > 0u);
    CHECK(cam.rgb_pixels().size() == static_cast<usize>(w) * static_cast<usize>(h) * 3u);
    CHECK(cam.rgba_pixels().size() == static_cast<usize>(w) * static_cast<usize>(h));

    const auto& stats = cam.frame_stats();
    u64 hist_sum = 0;
    for (int i = 0; i < 256; ++i) {
        hist_sum += stats.hist_luma[i];
    }
    CHECK(hist_sum == static_cast<u64>(w) * static_cast<u64>(h));
    CHECK(stats.mean_luminance > 5.0f && stats.mean_luminance < 250.0f);

    cam.set_torch_enabled(true);
    CHECK(cam.is_torch_enabled());
    CHECK(cam.capture_snapshot());
    CHECK(cam.captured_photo_count() == 1u);
    CHECK(cam.last_captured_rgb().size() == cam.rgb_pixels().size());

    if (cam.devices().size() > 1u) {
        const usize initial_idx = cam.active_device_index();
        CHECK(cam.switch_camera_facing());
        CHECK(cam.active_device_index() != initial_idx);
    }

    cam.shutdown();
}

void test_location() {
    // 1 degree of latitude along a meridian is ~111,195 m
    const GeoCoord a{0.0, 0.0};
    const GeoCoord b{1.0, 0.0};
    const f64 dist = LocationService::distance_meters(a, b);
    CHECK_NEAR(dist, 111195.0, 300.0);

    const f32 brg_north = LocationService::bearing_degrees(a, b);
    CHECK_NEAR(brg_north, 0.0f, 0.5f);

    const GeoCoord c{0.0, 1.0};
    const f32 brg_east = LocationService::bearing_degrees(a, c);
    CHECK_NEAR(brg_east, 90.0f, 0.5f);

    LocationService loc;
    CHECK(loc.initialize());

    for (int i = 0; i < 20; ++i) {
        loc.update_frame(0.15);
    }

    CHECK(loc.latest_fix().has_fix);
    CHECK(loc.breadcrumb_track().size() >= 2u);
    CHECK(loc.total_track_distance_meters() > 1.0);
    CHECK(loc.latest_fix().satellites_in_view >= 4);

    loc.clear_breadcrumb_track();
    CHECK(loc.breadcrumb_track().empty());
    loc.shutdown();
}

void test_audio() {
    AudioCapture audio;
    CHECK(audio.initialize(48000));
    audio.set_force_synthetic_signal(true);
    audio.set_synth_fundamental_hz(1000.0f);
    audio.set_synth_noise_floor_db(-60.0f);

    for (int i = 0; i < 6; ++i) {
        audio.update_frame(1.0 / 60.0);
    }

    CHECK(audio.rms_dbfs() > -30.0f);
    CHECK(audio.peak_dbfs() > -20.0f);
    // Peak frequency bin should be within ~1.5 FFT bins of 1000 Hz
    CHECK_NEAR(audio.dominant_freq_hz(), 1000.0f, 65.0f);
    CHECK(audio.waveform_samples().size() == AudioCapture::kWaveformWindowSamples);
    CHECK(audio.spectrum_slice_db().size() == AudioCapture::kSpectrumBins);

    audio.shutdown();
}

void test_system() {
    SystemBridge sys;
    CHECK(sys.initialize());
    CHECK(sys.permission_status(Permission::Camera) == PermissionStatus::Granted);

    bool callback_invoked = false;
    sys.request_permission(Permission::FineLocation, [&](Permission perm, PermissionStatus status) {
        callback_invoked = true;
        CHECK(perm == Permission::FineLocation);
        CHECK(status == PermissionStatus::Granted);
    });
    CHECK(callback_invoked);

    const u64 before = sys.haptic_pulse_count();
    sys.trigger_haptic(HapticStyle::Medium);
    CHECK(sys.haptic_pulse_count() == before + 1u);

    sys.set_simulate_mobile_cutout(true);
    CHECK(sys.safe_area_insets().top > 0.0f);
    sys.set_simulate_mobile_cutout(false);
    CHECK_NEAR(sys.safe_area_insets().top, 0.0f, 0.001f);

    sys.shutdown();
}

} // namespace

int main() {
    test_sensors();
    test_camera();
    test_location();
    test_audio();
    test_system();

    if (g_failures == 0) {
        keel::println("All keel_mobile tests passed.");
        return EXIT_SUCCESS;
    }
    keel::eprintln("%d test(s) failed.", g_failures);
    return EXIT_FAILURE;
}
