#pragma once

#include "keel/array.h"
#include "keel/mobile/types.h"
#include "keel/small_array.h"

namespace keel {

enum class CameraFacing : u8 {
    Back = 0, // Rear-facing primary/ultra-wide/telephoto camera
    Front,    // Front-facing selfie / FaceID camera
    External  // USB UVC or desktop webcam
};

enum class FlashMode : u8 {
    Off = 0,
    Auto,
    On,
    Torch     // Continuous LED flashlight illumination
};

struct CameraDeviceInfo {
    u32 device_index = 0;
    u32 platform_camera_id = 0;
    InplaceString<64> android_camera_id{};
    InplaceString<64> name{};
    CameraFacing facing = CameraFacing::Back;
    i32 sensor_orientation_deg = 90;
    bool has_flash_unit = false;
    i32 preferred_width = 640;
    i32 preferred_height = 480;
    f32 preferred_fps = 30.0f;
    bool is_synthetic = false;
};

struct CameraFrameStats {
    u32 width = 0;
    u32 height = 0;
    u64 frame_sequence = 0;
    f32 fps = 0.0f;
    f32 mean_luminance = 0.0f; // Average luminance [0..255]
    f32 contrast_rms = 0.0f;   // RMS contrast
    u32 hist_luma[256]{};      // 256-bin luminance histogram
    u32 hist_r[256]{};         // 256-bin Red histogram
    u32 hist_g[256]{};         // 256-bin Green histogram
    u32 hist_b[256]{};         // 256-bin Blue histogram
};

// Mobile Camera Service (Android Camera2 NDK + Procedural Optical Test Camera + External Frame Injection).
class CameraService {
public:
    CameraService();
    ~CameraService();

    CameraService(const CameraService&) = delete;
    CameraService& operator=(const CameraService&) = delete;

    bool initialize();
    void shutdown();

    void register_external_device(const CameraDeviceInfo& dev);
    void push_external_rgb_frame(ArrayRef<u8> rgb, u32 width, u32 height, usize row_stride = 0);

    bool open_camera(usize device_index = 0, i32 width = 640, i32 height = 480);
    void close_camera();
    [[nodiscard]] bool is_streaming() const noexcept { return m_streaming; }

    void set_paused(bool paused);
    [[nodiscard]] bool is_paused() const noexcept { return m_paused; }

    bool switch_camera_facing();

    void set_display_rotation_degrees(i32 degrees) noexcept { m_display_rotation_deg = degrees; }
    [[nodiscard]] i32 display_rotation_degrees() const noexcept { return m_display_rotation_deg; }

    void update_frame(f64 dt_seconds);

    [[nodiscard]] ArrayRef<CameraDeviceInfo> devices() const noexcept { return m_devices; }
    [[nodiscard]] usize active_device_index() const noexcept { return m_active_device_index; }
    [[nodiscard]] const CameraDeviceInfo* active_device() const noexcept;

    [[nodiscard]] u32 frame_width() const noexcept { return m_frame_width; }
    [[nodiscard]] u32 frame_height() const noexcept { return m_frame_height; }
    [[nodiscard]] ArrayRef<u8> rgb_pixels() const noexcept { return m_rgb_pixels; }
    [[nodiscard]] ArrayRef<u32> rgba_pixels() const noexcept { return m_rgba_pixels; }
    [[nodiscard]] const CameraFrameStats& frame_stats() const noexcept { return m_stats; }

    void set_flash_mode(FlashMode mode);
    [[nodiscard]] FlashMode flash_mode() const noexcept { return m_flash_mode; }

    void set_torch_enabled(bool enabled);
    [[nodiscard]] bool is_torch_enabled() const noexcept { return m_torch_enabled; }

    void trigger_tap_to_focus(Vec2f normalized_point) noexcept;
    [[nodiscard]] Vec2f focus_point() const noexcept { return m_focus_point; }

    void set_exposure_compensation_ev(f32 ev) noexcept;
    [[nodiscard]] f32 exposure_compensation_ev() const noexcept { return m_exposure_ev; }

    void set_zoom_factor(f32 zoom) noexcept;
    [[nodiscard]] f32 zoom_factor() const noexcept { return m_zoom_factor; }

    bool capture_snapshot();
    [[nodiscard]] u32 captured_photo_count() const noexcept { return m_captured_photos; }
    [[nodiscard]] ArrayRef<u8> last_captured_rgb() const noexcept { return m_last_captured_rgb; }

private:
    void generate_procedural_frame(f64 dt_seconds);
    void compute_frame_histograms();
    bool open_android_camera(const CameraDeviceInfo& dev, i32 width, i32 height);
    void close_android_camera();

    struct AndroidCameraState;
    AndroidCameraState* m_android_cam = nullptr;

    SmallArray<CameraDeviceInfo, 4> m_devices{};
    usize m_active_device_index = 0;
    i32 m_display_rotation_deg = 0;

    u32 m_frame_width = 480;
    u32 m_frame_height = 640;
    Array<u8> m_rgb_pixels{};
    Array<u32> m_rgba_pixels{};
    Array<u8> m_last_captured_rgb{};
    CameraFrameStats m_stats{};

    bool m_initialized = false;
    bool m_streaming = false;
    bool m_paused = false;
    bool m_using_synthetic = false;
    bool m_got_external_frame = false;
    bool m_torch_enabled = false;
    FlashMode m_flash_mode = FlashMode::Off;
    Vec2f m_focus_point{0.5f, 0.5f};
    f32 m_exposure_ev = 0.0f;
    f32 m_zoom_factor = 1.0f;
    u32 m_captured_photos = 0;
    f64 m_sim_time_sec = 0.0;
    f64 m_retry_timer_sec = 0.0;
    f64 m_fps_accum_sec = 0.0;
    u32 m_fps_frame_count = 0;
};

} // namespace keel
