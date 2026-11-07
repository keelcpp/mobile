#pragma once

#include "keel/array.h"
#include "keel/mobile/types.h"

namespace keel {

// Single GNSS / Fused Location fix (GPS / GLONASS / Galileo / Network).
struct LocationFix {
    GeoCoord coord{59.9343, 30.3351};   // Latitude & Longitude in WGS-84 degrees
    f32 altitude_meters = 14.0f;        // Altitude above WGS-84 ellipsoid / MSL in meters
    f32 horizontal_accuracy_m = 3.8f;   // 1-sigma horizontal accuracy radius in meters
    f32 vertical_accuracy_m = 5.2f;     // Vertical accuracy in meters
    f32 speed_mps = 0.0f;               // Ground speed in m/s
    f32 course_deg = 0.0f;              // Ground track / bearing in degrees [0..360)
    i32 satellites_in_view = 18;
    i32 satellites_used = 12;
    u64 timestamp_ms = 0;
    bool has_fix = true;
    bool is_mock = false;

    [[nodiscard]] constexpr f32 speed_knots() const noexcept { return speed_mps * 1.943844f; }
    [[nodiscard]] constexpr f32 speed_kmh() const noexcept { return speed_mps * 3.6f; }
};

// Mobile GNSS / Location service (Android LocationManager JNI + Desktop Route Simulator).
class LocationService {
public:
    static constexpr usize kMaxTrackHistory = 512;

    LocationService();
    ~LocationService();

    LocationService(const LocationService&) = delete;
    LocationService& operator=(const LocationService&) = delete;

    bool initialize(void* jni_env = nullptr, void* android_activity = nullptr);
    void shutdown();

    void set_android_context(void* jni_env, void* android_activity) noexcept {
        m_jni_env = jni_env;
        m_android_activity = android_activity;
    }

    void set_enabled(bool enabled) noexcept { m_enabled = enabled; }
    [[nodiscard]] bool is_enabled() const noexcept { return m_enabled; }

    void update_frame(f64 dt_seconds);

    [[nodiscard]] const LocationFix& latest_fix() const noexcept { return m_latest_fix; }
    [[nodiscard]] ArrayRef<GeoCoord> breadcrumb_track() const noexcept { return m_track_history; }

    void clear_breadcrumb_track() noexcept { m_track_history.clear(); }
    void set_simulated_route_center(GeoCoord center) noexcept;

    [[nodiscard]] static f64 distance_meters(GeoCoord a, GeoCoord b) noexcept;
    [[nodiscard]] static f32 bearing_degrees(GeoCoord a, GeoCoord b) noexcept;

    [[nodiscard]] f64 total_track_distance_meters() const noexcept { return m_total_distance_meters; }

private:
    bool poll_android_location();
    void step_simulated_route(f64 dt_seconds);
    void append_track_point(GeoCoord pt);

    void* m_jni_env = nullptr;
    void* m_android_activity = nullptr;
    LocationFix m_latest_fix{};
    GeoCoord m_sim_center{44.6020, 33.4950};
    Array<GeoCoord> m_track_history{};
    f64 m_total_distance_meters = 0.0;
    f64 m_sim_time_sec = 0.0;
    f64 m_track_sample_timer_sec = 0.0;
    bool m_initialized = false;
    bool m_enabled = true;
    bool m_has_android_location = false;
};

} // namespace keel
