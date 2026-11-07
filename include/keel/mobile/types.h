#pragma once

#include "keel/core.h"
#include "keel/string.h"

namespace keel {

struct Vec2f {
    f32 x = 0.0f;
    f32 y = 0.0f;

    constexpr Vec2f() noexcept = default;
    constexpr Vec2f(f32 x_, f32 y_) noexcept : x(x_), y(y_) {}
};

// 3D vector used for 3-axis physical sensors (Accelerometer, Gyroscope, Magnetometer).
struct Vec3f {
    f32 x = 0.0f;
    f32 y = 0.0f;
    f32 z = 0.0f;

    constexpr Vec3f() noexcept = default;
    constexpr Vec3f(f32 x_, f32 y_, f32 z_) noexcept : x(x_), y(y_), z(z_) {}

    [[nodiscard]] constexpr f32 length_sq() const noexcept {
        return x * x + y * y + z * z;
    }

    [[nodiscard]] f32 magnitude() const noexcept;
};

// Unit quaternion representing 3D device orientation (Rotation Vector / Attitude).
struct Quaternion {
    f32 w = 1.0f;
    f32 x = 0.0f;
    f32 y = 0.0f;
    f32 z = 0.0f;

    constexpr Quaternion() noexcept = default;
    constexpr Quaternion(f32 w_, f32 x_, f32 y_, f32 z_) noexcept
        : w(w_), x(x_), y(y_), z(z_) {}
};

// Euler orientation angles in degrees derived from fused IMU sensors.
struct EulerAngles {
    f32 pitch_deg = 0.0f;   // Nose up/down [-90..+90]
    f32 roll_deg = 0.0f;    // Bank left/right [-180..+180]
    f32 yaw_deg = 0.0f;     // Azimuth / Yaw [-180..+180]
    f32 heading_deg = 0.0f; // Magnetic / True compass heading [0..360)
};

// WGS-84 geodetic coordinate in degrees.
struct GeoCoord {
    f64 lat = 0.0;
    f64 lon = 0.0;

    constexpr GeoCoord() noexcept = default;
    constexpr GeoCoord(f64 lat_, f64 lon_) noexcept : lat(lat_), lon(lon_) {}
};

// Display safe-area insets in logical points (status bar, camera cutout/notch, gesture bar).
struct SafeAreaInsets {
    f32 top = 0.0f;
    f32 right = 0.0f;
    f32 bottom = 0.0f;
    f32 left = 0.0f;
};

// Device screen orientation modes.
enum class ScreenOrientation : u8 {
    Portrait = 0,
    LandscapeLeft,
    PortraitUpsideDown,
    LandscapeRight,
    Auto
};

// Mobile runtime permissions (mapped to Android Manifest permissions & iOS Info.plist keys).
enum class Permission : u8 {
    Camera = 0,
    RecordAudio,
    FineLocation,
    CoarseLocation,
    BodySensors,
    Bluetooth,
    Notifications,
    Count
};

// Authorization status of a runtime permission.
enum class PermissionStatus : u8 {
    Unknown = 0,
    Granted,
    Denied,
    Restricted
};

// Standard mobile tactile haptic feedback patterns.
enum class HapticStyle : u8 {
    Light = 0,
    Medium,
    Heavy,
    Selection,
    Success,
    Warning,
    Error
};

// Real-time device battery and thermal telemetry.
struct BatteryStatus {
    f32 level_pct = 100.0f;    // Battery charge [0..100]%
    bool is_charging = false;  // True when plugged into AC/USB/Wireless charger
    f32 temperature_c = 28.5f; // Battery temperature in Celsius
    f32 voltage_mv = 4150.0f;  // Battery voltage in millivolts
};

// Summary information about the host mobile device or simulator.
struct DeviceInfo {
    InplaceString<64> platform_name{"Android"};
    InplaceString<64> manufacturer{"Generic"};
    InplaceString<64> model{"Mobile Device"};
    InplaceString<64> os_version{"14"};
    i32 sdk_int = 34;
    bool is_simulator = false;
};

} // namespace keel
