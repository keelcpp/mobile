#include "keel/mobile/system.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>

#if defined(__ANDROID__)
#include <android/log.h>
#include <jni.h>
#include <sys/system_properties.h>
#define KEEL_SYS_LOGI(...) __android_log_print(ANDROID_LOG_INFO, "keel::System", __VA_ARGS__)
#define KEEL_SYS_LOGW(...) __android_log_print(ANDROID_LOG_WARN, "keel::System", __VA_ARGS__)
#endif

namespace keel {

SystemBridge::SystemBridge() {
    for (usize i = 0; i < kPermissionCount; ++i) {
        m_permissions[i] = PermissionStatus::Granted;
    }
}

SystemBridge::~SystemBridge() {
    shutdown();
}

bool SystemBridge::initialize(void* jni_env, void* android_activity) {
    if (m_initialized) return true;

    if (jni_env) m_jni_env = jni_env;
    if (android_activity) m_android_activity = android_activity;

    query_android_device_info();
    apply_simulated_cutout_if_needed();

    for (usize i = 0; i < kPermissionCount; ++i) {
        m_permissions[i] = PermissionStatus::Granted;
    }

    m_initialized = true;
    return true;
}

void SystemBridge::shutdown() {
    m_initialized = false;
}

void SystemBridge::query_android_device_info() {
#if defined(__ANDROID__)
    m_device_info.platform_name = InplaceString<64>("Android");
    m_device_info.is_simulator = false;

    char prop_buf[PROP_VALUE_MAX] = {0};
    if (__system_property_get("ro.product.manufacturer", prop_buf) > 0) {
        m_device_info.manufacturer = InplaceString<64>(StringRef(prop_buf));
    }
    if (__system_property_get("ro.product.model", prop_buf) > 0) {
        m_device_info.model = InplaceString<64>(StringRef(prop_buf));
    }
    if (__system_property_get("ro.build.version.release", prop_buf) > 0) {
        m_device_info.os_version = InplaceString<64>(StringRef(prop_buf));
    }
    if (__system_property_get("ro.build.version.sdk", prop_buf) > 0) {
        m_device_info.sdk_int = std::atoi(prop_buf);
    }
    KEEL_SYS_LOGI("Device: %s %s (Android %s, SDK %d)",
                  m_device_info.manufacturer.c_str(),
                  m_device_info.model.c_str(),
                  m_device_info.os_version.c_str(),
                  m_device_info.sdk_int);
#else
#if defined(_WIN32)
    m_device_info.platform_name = InplaceString<64>("Windows (Mobile Sim)");
#elif defined(__APPLE__)
    m_device_info.platform_name = InplaceString<64>("macOS / iOS (Mobile Sim)");
#else
    m_device_info.platform_name = InplaceString<64>("Linux (Mobile Sim)");
#endif
    m_device_info.manufacturer = InplaceString<64>("Keel");
    m_device_info.model = InplaceString<64>("Pro Mobile Simulator");
    m_device_info.os_version = InplaceString<64>("15.0");
    m_device_info.sdk_int = 35;
    m_device_info.is_simulator = true;
#endif
}

void SystemBridge::apply_simulated_cutout_if_needed() noexcept {
    if (m_simulate_cutout) {
        if (m_screen_orientation == ScreenOrientation::Portrait ||
            m_screen_orientation == ScreenOrientation::PortraitUpsideDown) {
            m_safe_area = SafeAreaInsets{28.0f, 0.0f, 16.0f, 0.0f};
        } else {
            m_safe_area = SafeAreaInsets{0.0f, 28.0f, 14.0f, 28.0f};
        }
    } else {
        m_safe_area = SafeAreaInsets{0.0f, 0.0f, 0.0f, 0.0f};
    }
}

void SystemBridge::set_simulate_mobile_cutout(bool enabled) noexcept {
    m_simulate_cutout = enabled;
    apply_simulated_cutout_if_needed();
}

void SystemBridge::update_frame(f64 dt_seconds) {
    m_poll_timer_sec += dt_seconds;
    if (m_poll_timer_sec < 1.0) return;
    m_poll_timer_sec = 0.0;

    // Gentle battery temperature drift in simulator
    if (m_device_info.is_simulator) {
        m_battery.temperature_c = 29.0f + 1.5f * std::sin(static_cast<f32>(m_haptic_count) * 0.2f);
    }
}

PermissionStatus SystemBridge::permission_status(Permission perm) const noexcept {
    const usize idx = static_cast<usize>(perm);
    if (idx >= kPermissionCount) return PermissionStatus::Denied;
    return m_permissions[idx];
}

void SystemBridge::set_permission_status(Permission perm, PermissionStatus status) noexcept {
    const usize idx = static_cast<usize>(perm);
    if (idx < kPermissionCount) {
        m_permissions[idx] = status;
    }
}

const char* SystemBridge::android_permission_string(Permission perm) noexcept {
    switch (perm) {
        case Permission::Camera:         return "android.permission.CAMERA";
        case Permission::RecordAudio:    return "android.permission.RECORD_AUDIO";
        case Permission::FineLocation:   return "android.permission.ACCESS_FINE_LOCATION";
        case Permission::CoarseLocation: return "android.permission.ACCESS_COARSE_LOCATION";
        case Permission::BodySensors:    return "android.permission.BODY_SENSORS";
        case Permission::Bluetooth:      return "android.permission.BLUETOOTH_CONNECT";
        case Permission::Notifications:  return "android.permission.POST_NOTIFICATIONS";
        default:                         return "";
    }
}

StringRef SystemBridge::permission_display_name(Permission perm) noexcept {
    switch (perm) {
        case Permission::Camera:         return "Camera";
        case Permission::RecordAudio:    return "Microphone";
        case Permission::FineLocation:   return "Precise Location (GNSS)";
        case Permission::CoarseLocation: return "Approximate Location";
        case Permission::BodySensors:    return "Body & Motion Sensors";
        case Permission::Bluetooth:      return "Bluetooth Devices";
        case Permission::Notifications:  return "Notifications";
        default:                         return "Unknown";
    }
}

void SystemBridge::request_permission(Permission perm, PermissionCallback callback) {
    const usize idx = static_cast<usize>(perm);
    if (idx >= kPermissionCount) {
        if (callback) callback(perm, PermissionStatus::Denied);
        return;
    }

    m_permissions[idx] = PermissionStatus::Granted;
    if (callback) {
        callback(perm, PermissionStatus::Granted);
    }
}

void SystemBridge::trigger_haptic(HapticStyle style) {
    if (!m_haptics_enabled) return;

    u32 duration_ms = 12;
    switch (style) {
        case HapticStyle::Selection: duration_ms = 8;  break;
        case HapticStyle::Light:     duration_ms = 14; break;
        case HapticStyle::Medium:    duration_ms = 25; break;
        case HapticStyle::Heavy:     duration_ms = 45; break;
        case HapticStyle::Success:   duration_ms = 30; break;
        case HapticStyle::Warning:   duration_ms = 40; break;
        case HapticStyle::Error:     duration_ms = 60; break;
    }
    vibrate_ms(duration_ms);
}

void SystemBridge::vibrate_ms(u32 duration_ms) {
    if (!m_haptics_enabled || duration_ms == 0) return;
    m_haptic_count++;

#if defined(__ANDROID__)
    if (m_jni_env && m_android_activity) {
        auto* env = static_cast<JNIEnv*>(m_jni_env);
        auto activity = static_cast<jobject>(m_android_activity);
        jclass activity_cls = env->GetObjectClass(activity);
        if (activity_cls) {
            jmethodID get_service = env->GetMethodID(
                activity_cls, "getSystemService", "(Ljava/lang/String;)Ljava/lang/Object;");
            if (get_service) {
                jstring service_name = env->NewStringUTF("vibrator");
                jobject vibrator = env->CallObjectMethod(activity, get_service, service_name);
                env->DeleteLocalRef(service_name);
                if (vibrator) {
                    jclass vib_cls = env->GetObjectClass(vibrator);
                    if (vib_cls) {
                        jmethodID vib_method = env->GetMethodID(vib_cls, "vibrate", "(J)V");
                        if (vib_method) {
                            env->CallVoidMethod(vibrator, vib_method, static_cast<jlong>(duration_ms));
                        }
                        env->DeleteLocalRef(vib_cls);
                    }
                    env->DeleteLocalRef(vibrator);
                }
            }
            env->DeleteLocalRef(activity_cls);
        }
        if (env->ExceptionCheck()) {
            env->ExceptionClear();
        }
    }
#endif
}

} // namespace keel
