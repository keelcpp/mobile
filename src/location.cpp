#include "keel/mobile/location.h"
#include "keel/algo.h"

#include <chrono>
#include <cmath>
#include <numbers>

#ifdef __ANDROID__
#include <jni.h>
#endif

namespace keel {

namespace {

constexpr f64 kEarthRadiusMeters = 6371008.8;
constexpr f64 kDegToRad = std::numbers::pi / 180.0;
constexpr f64 kRadToDeg = 180.0 / std::numbers::pi;

[[nodiscard]] u64 now_steady_ms() noexcept {
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()
    ).count();
    return static_cast<u64>(keel::max<i64>(0, ms));
}

} // namespace

LocationService::LocationService() {
    m_track_history.reserve(kMaxTrackHistory);
}

LocationService::~LocationService() {
    shutdown();
}

bool LocationService::initialize(void* jni_env, void* android_activity) {
    if (jni_env) m_jni_env = jni_env;
    if (android_activity) m_android_activity = android_activity;
    if (m_initialized) return true;

    m_latest_fix.coord = m_sim_center;
    m_latest_fix.altitude_meters = 18.5f;
    m_latest_fix.horizontal_accuracy_m = 3.2f;
    m_latest_fix.vertical_accuracy_m = 4.8f;
    m_latest_fix.speed_mps = 6.4f;
    m_latest_fix.course_deg = 45.0f;
    m_latest_fix.satellites_in_view = 22;
    m_latest_fix.satellites_used = 14;
    m_latest_fix.has_fix = true;
    m_latest_fix.is_mock = true;

    m_track_history.clear();
    m_total_distance_meters = 0.0;

    for (int i = 0; i < 24; ++i) {
        step_simulated_route(0.25);
    }

    m_initialized = true;
    return true;
}

void LocationService::shutdown() {
    m_initialized = false;
}

void LocationService::set_simulated_route_center(GeoCoord center) noexcept {
    m_sim_center = center;
    m_track_history.clear();
    m_total_distance_meters = 0.0;
    m_latest_fix.coord = center;
}

void LocationService::update_frame(f64 dt_seconds) {
    if (!m_enabled) return;

    const f64 clamped_dt = keel::clamp(dt_seconds, 0.001, 0.25);
    if (!poll_android_location()) {
        step_simulated_route(clamped_dt);
    }
}

bool LocationService::poll_android_location() {
#ifdef __ANDROID__
    auto* env = static_cast<JNIEnv*>(m_jni_env);
    jobject activity = static_cast<jobject>(m_android_activity);
    if (!env || !activity) return false;

    bool got_fix = false;
    jclass activity_class = env->GetObjectClass(activity);
    jmethodID get_system_service = env->GetMethodID(
        activity_class,
        "getSystemService",
        "(Ljava/lang/String;)Ljava/lang/Object;"
    );

    if (get_system_service) {
        jstring loc_str = env->NewStringUTF("location");
        jobject loc_manager = env->CallObjectMethod(activity, get_system_service, loc_str);
        env->DeleteLocalRef(loc_str);

        if (!env->ExceptionCheck() && loc_manager) {
            jclass lm_class = env->GetObjectClass(loc_manager);
            jmethodID get_last_known = env->GetMethodID(
                lm_class,
                "getLastKnownLocation",
                "(Ljava/lang/String;)Landroid/location/Location;"
            );

            if (get_last_known) {
                jstring gps_provider = env->NewStringUTF("gps");
                jobject location_obj = env->CallObjectMethod(loc_manager, get_last_known, gps_provider);
                env->DeleteLocalRef(gps_provider);

                if (env->ExceptionCheck()) {
                    env->ExceptionClear();
                    location_obj = nullptr;
                }

                if (!location_obj) {
                    jstring net_provider = env->NewStringUTF("network");
                    location_obj = env->CallObjectMethod(loc_manager, get_last_known, net_provider);
                    env->DeleteLocalRef(net_provider);
                    if (env->ExceptionCheck()) {
                        env->ExceptionClear();
                        location_obj = nullptr;
                    }
                }

                if (location_obj) {
                    jclass loc_class = env->GetObjectClass(location_obj);
                    jmethodID get_lat = env->GetMethodID(loc_class, "getLatitude", "()D");
                    jmethodID get_lon = env->GetMethodID(loc_class, "getLongitude", "()D");
                    jmethodID get_alt = env->GetMethodID(loc_class, "getAltitude", "()D");
                    jmethodID get_acc = env->GetMethodID(loc_class, "getAccuracy", "()F");
                    jmethodID get_spd = env->GetMethodID(loc_class, "getSpeed", "()F");
                    jmethodID get_brg = env->GetMethodID(loc_class, "getBearing", "()F");

                    const f64 lat = env->CallDoubleMethod(location_obj, get_lat);
                    const f64 lon = env->CallDoubleMethod(location_obj, get_lon);
                    const f64 alt = env->CallDoubleMethod(location_obj, get_alt);
                    const f32 acc = env->CallFloatMethod(location_obj, get_acc);
                    const f32 spd = env->CallFloatMethod(location_obj, get_spd);
                    const f32 brg = env->CallFloatMethod(location_obj, get_brg);

                    m_latest_fix.coord = {lat, lon};
                    m_latest_fix.altitude_meters = static_cast<f32>(alt);
                    m_latest_fix.horizontal_accuracy_m = keel::max(1.0f, acc);
                    m_latest_fix.speed_mps = spd;
                    m_latest_fix.course_deg = brg;
                    m_latest_fix.timestamp_ms = now_steady_ms();
                    m_latest_fix.has_fix = true;
                    m_latest_fix.is_mock = false;
                    m_has_android_location = true;
                    append_track_point(m_latest_fix.coord);
                    got_fix = true;

                    env->DeleteLocalRef(loc_class);
                    env->DeleteLocalRef(location_obj);
                }
            }
            env->DeleteLocalRef(lm_class);
            env->DeleteLocalRef(loc_manager);
        } else if (env->ExceptionCheck()) {
            env->ExceptionClear();
        }
    }

    env->DeleteLocalRef(activity_class);
    return got_fix;
#else
    return false;
#endif
}

void LocationService::step_simulated_route(f64 dt_seconds) {
    m_sim_time_sec += dt_seconds;
    m_track_sample_timer_sec += dt_seconds;

    const f64 omega = 0.085;
    const f64 phase = m_sim_time_sec * omega;

    const f64 d_lat = 0.011 * std::sin(phase);
    const f64 d_lon = 0.018 * std::sin(phase * 0.5) * std::cos(phase * 0.5);

    const GeoCoord prev_coord = m_latest_fix.coord;
    const GeoCoord new_coord{
        m_sim_center.lat + d_lat,
        m_sim_center.lon + d_lon
    };

    const f64 step_dist = distance_meters(prev_coord, new_coord);
    if (dt_seconds > 0.001 && step_dist > 0.01) {
        m_latest_fix.speed_mps = static_cast<f32>(keel::clamp(step_dist / dt_seconds, 0.0, 45.0));
        m_latest_fix.course_deg = bearing_degrees(prev_coord, new_coord);
    }

    m_latest_fix.coord = new_coord;
    m_latest_fix.altitude_meters = 16.0f + 4.5f * static_cast<f32>(std::sin(m_sim_time_sec * 0.2));
    m_latest_fix.horizontal_accuracy_m = 2.8f + 0.6f * static_cast<f32>(std::cos(m_sim_time_sec * 0.35));
    m_latest_fix.timestamp_ms = now_steady_ms();
    m_latest_fix.has_fix = true;
    m_latest_fix.is_mock = true;

    if (m_track_history.empty() || m_track_sample_timer_sec >= 0.20) {
        m_track_sample_timer_sec = 0.0;
        append_track_point(new_coord);
    }
}

void LocationService::append_track_point(GeoCoord pt) {
    if (!m_track_history.empty()) {
        const f64 d = distance_meters(m_track_history.back(), pt);
        if (d < 1.5) return;
        m_total_distance_meters += d;
    }

    if (m_track_history.size() < kMaxTrackHistory) {
        m_track_history.push_back(pt);
    } else {
        mem_move(m_track_history.data(), m_track_history.data() + 1, (m_track_history.size() - 1) * sizeof(GeoCoord));
        m_track_history.back() = pt;
    }
}

f64 LocationService::distance_meters(GeoCoord a, GeoCoord b) noexcept {
    const f64 lat1 = a.lat * kDegToRad;
    const f64 lat2 = b.lat * kDegToRad;
    const f64 d_lat = (b.lat - a.lat) * kDegToRad;
    const f64 d_lon = (b.lon - a.lon) * kDegToRad;

    const f64 sin_lat = std::sin(d_lat * 0.5);
    const f64 sin_lon = std::sin(d_lon * 0.5);
    const f64 h = sin_lat * sin_lat + std::cos(lat1) * std::cos(lat2) * sin_lon * sin_lon;
    return 2.0 * kEarthRadiusMeters * std::asin(keel::min(1.0, std::sqrt(h)));
}

f32 LocationService::bearing_degrees(GeoCoord a, GeoCoord b) noexcept {
    const f64 lat1 = a.lat * kDegToRad;
    const f64 lat2 = b.lat * kDegToRad;
    const f64 d_lon = (b.lon - a.lon) * kDegToRad;

    const f64 y = std::sin(d_lon) * std::cos(lat2);
    const f64 x = std::cos(lat1) * std::sin(lat2) -
                  std::sin(lat1) * std::cos(lat2) * std::cos(d_lon);
    const f64 brg = std::fmod(std::atan2(y, x) * kRadToDeg + 360.0, 360.0);
    return static_cast<f32>(brg);
}

} // namespace keel
