#pragma once

#include "keel/function.h"
#include "keel/mobile/types.h"

namespace keel {

// System-level bridge for Mobile OS features:
// - Runtime Permissions (Android Activity.checkSelfPermission / requestPermissions JNI + Simulator)
// - Haptic Feedback & Vibration (Android JNI Vibrator + Simulator)
// - Display Safe Area Insets & Screen Orientation
// - Battery & Power telemetry (Android JNI BatteryManager + Windows GetSystemPowerStatus + Simulator)
// - Native OS Toast notifications (Android JNI android.widget.Toast)
class SystemBridge {
public:
    static constexpr usize kPermissionCount = static_cast<usize>(Permission::Count);
    using PermissionCallback = Function<void(Permission, PermissionStatus)>;

    SystemBridge();
    ~SystemBridge();

    SystemBridge(const SystemBridge&) = delete;
    SystemBridge& operator=(const SystemBridge&) = delete;

    bool initialize(void* jni_env = nullptr, void* android_activity = nullptr);
    void shutdown();

    void set_android_context(void* jni_env, void* android_activity) noexcept {
        m_jni_env = jni_env;
        m_android_activity = android_activity;
    }

    void update_frame(f64 dt_seconds);

    // Device & OS Info
    [[nodiscard]] const DeviceInfo& device_info() const noexcept { return m_device_info; }
    [[nodiscard]] DeviceInfo& mutable_device_info() noexcept { return m_device_info; }

    // Runtime Permissions
    [[nodiscard]] PermissionStatus permission_status(Permission perm) const noexcept;
    void set_permission_status(Permission perm, PermissionStatus status) noexcept;
    void request_permission(Permission perm, PermissionCallback callback = {});
    [[nodiscard]] static const char* android_permission_string(Permission perm) noexcept;
    [[nodiscard]] static StringRef permission_display_name(Permission perm) noexcept;

    // Haptics & Tactile Feedback
    void trigger_haptic(HapticStyle style = HapticStyle::Light);
    void vibrate_ms(u32 duration_ms);
    void set_haptics_enabled(bool enabled) noexcept { m_haptics_enabled = enabled; }
    [[nodiscard]] bool is_haptics_enabled() const noexcept { return m_haptics_enabled; }
    [[nodiscard]] u64 haptic_pulse_count() const noexcept { return m_haptic_count; }

    // Safe Area Insets & Screen Orientation
    [[nodiscard]] SafeAreaInsets safe_area_insets() const noexcept { return m_safe_area; }
    void set_safe_area_insets(SafeAreaInsets insets) noexcept { m_safe_area = insets; }
    [[nodiscard]] ScreenOrientation screen_orientation() const noexcept { return m_screen_orientation; }
    void set_screen_orientation(ScreenOrientation orientation) noexcept { m_screen_orientation = orientation; }
    void set_simulate_mobile_cutout(bool enabled) noexcept;
    [[nodiscard]] bool is_simulating_mobile_cutout() const noexcept { return m_simulate_cutout; }

    // Battery & Thermal
    [[nodiscard]] const BatteryStatus& battery_status() const noexcept { return m_battery; }
    void set_battery_status(const BatteryStatus& status) noexcept { m_battery = status; }

    // Native OS Toast
    void show_native_toast(StringRef message, bool long_duration = false);

private:
    void query_android_device_info();
    void poll_battery_status();
    void refresh_android_permissions();
    void apply_simulated_cutout_if_needed() noexcept;

    void* m_jni_env = nullptr;
    void* m_android_activity = nullptr;
    DeviceInfo m_device_info{};
    SafeAreaInsets m_safe_area{};
    ScreenOrientation m_screen_orientation = ScreenOrientation::Portrait;
    BatteryStatus m_battery{};
    PermissionStatus m_permissions[kPermissionCount]{};

    bool m_initialized = false;
    bool m_haptics_enabled = true;
    bool m_simulate_cutout = true;
    u64 m_haptic_count = 0;
    f64 m_poll_timer_sec = 0.0;
};

} // namespace keel
