#include "keel/mobile/camera.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <mutex>

#if defined(__ANDROID__)
#include <android/log.h>
#include <android/native_window.h>
#include <camera/NdkCameraCaptureSession.h>
#include <camera/NdkCameraDevice.h>
#include <camera/NdkCameraError.h>
#include <camera/NdkCameraManager.h>
#include <camera/NdkCameraMetadata.h>
#include <media/NdkImage.h>
#include <media/NdkImageReader.h>
#define KEEL_CAM_LOGI(...) __android_log_print(ANDROID_LOG_INFO, "keel::Camera", __VA_ARGS__)
#define KEEL_CAM_LOGW(...) __android_log_print(ANDROID_LOG_WARN, "keel::Camera", __VA_ARGS__)
#define KEEL_CAM_LOGE(...) __android_log_print(ANDROID_LOG_ERROR, "keel::Camera", __VA_ARGS__)
#endif

namespace keel {

namespace {
constexpr f64 kPi = 3.14159265358979323846;

inline u8 clamp_byte(i32 v) noexcept {
    if (v < 0) return 0;
    if (v > 255) return 255;
    return static_cast<u8>(v);
}

inline u8 clamp_byte_f32(f32 v) noexcept {
    if (v <= 0.0f) return 0;
    if (v >= 255.0f) return 255;
    return static_cast<u8>(v + 0.5f);
}
} // namespace

#if defined(__ANDROID__)
struct CameraService::AndroidCameraState {
    ACameraManager* manager = nullptr;
    ACameraDevice* device = nullptr;
    AImageReader* image_reader = nullptr;
    ANativeWindow* reader_window = nullptr;
    ACameraOutputTarget* output_target = nullptr;
    ACaptureRequest* capture_request = nullptr;
    ACaptureSessionOutputContainer* session_outputs = nullptr;
    ACaptureSessionOutput* session_output = nullptr;
    ACameraCaptureSession* capture_session = nullptr;

    std::mutex frame_mutex;
    Array<u8> latest_rgb;
    u32 latest_width = 0;
    u32 latest_height = 0;
    std::atomic<bool> has_new_frame{false};
    std::atomic<bool> device_disconnected{false};

    i32 sensor_orientation_deg = 90;
    CameraFacing facing = CameraFacing::Back;
    InplaceString<64> camera_id{};
    bool torch_supported = false;

    static void on_device_disconnected(void* context, ACameraDevice* /*device*/) {
        auto* self = static_cast<AndroidCameraState*>(context);
        if (self) {
            self->device_disconnected.store(true, std::memory_order_release);
        }
    }

    static void on_device_error(void* context, ACameraDevice* /*device*/, int error) {
        auto* self = static_cast<AndroidCameraState*>(context);
        KEEL_CAM_LOGW("Camera2 device error: %d", error);
        if (self) {
            self->device_disconnected.store(true, std::memory_order_release);
        }
    }

    static void on_session_closed(void* /*context*/, ACameraCaptureSession* /*session*/) {}
    static void on_session_ready(void* /*context*/, ACameraCaptureSession* /*session*/) {}
    static void on_session_active(void* /*context*/, ACameraCaptureSession* /*session*/) {}

    static void on_image_available(void* context, AImageReader* reader) {
        auto* self = static_cast<AndroidCameraState*>(context);
        if (!self || !reader) return;

        AImage* image = nullptr;
        media_status_t status = AImageReader_acquireLatestImage(reader, &image);
        if (status != AMEDIA_OK || !image) {
            return;
        }

        int32_t src_w = 0;
        int32_t src_h = 0;
        AImage_getWidth(image, &src_w);
        AImage_getHeight(image, &src_h);

        uint8_t* y_data = nullptr;
        uint8_t* u_data = nullptr;
        uint8_t* v_data = nullptr;
        int y_len = 0, u_len = 0, v_len = 0;
        int32_t y_row_stride = 0, uv_row_stride = 0;
        int32_t uv_pixel_stride = 1;

        if (AImage_getPlaneData(image, 0, &y_data, &y_len) == AMEDIA_OK &&
            AImage_getPlaneData(image, 1, &u_data, &u_len) == AMEDIA_OK &&
            AImage_getPlaneData(image, 2, &v_data, &v_len) == AMEDIA_OK &&
            y_data && u_data && v_data && src_w > 0 && src_h > 0) {
            AImage_getPlaneRowStride(image, 0, &y_row_stride);
            AImage_getPlaneRowStride(image, 1, &uv_row_stride);
            AImage_getPlanePixelStride(image, 1, &uv_pixel_stride);

            const i32 rot = ((self->sensor_orientation_deg % 360) + 360) % 360;
            const bool swap_wh = (rot == 90 || rot == 270);
            const u32 dst_w = static_cast<u32>(swap_wh ? src_h : src_w);
            const u32 dst_h = static_cast<u32>(swap_wh ? src_w : src_h);
            const bool mirror_x = (self->facing == CameraFacing::Front);

            Array<u8> rgb(static_cast<usize>(dst_w) * static_cast<usize>(dst_h) * 3u);

            for (int32_t sy = 0; sy < src_h; ++sy) {
                const uint8_t* y_row = y_data + sy * y_row_stride;
                const uint8_t* u_row = u_data + (sy >> 1) * uv_row_stride;
                const uint8_t* v_row = v_data + (sy >> 1) * uv_row_stride;

                for (int32_t sx = 0; sx < src_w; ++sx) {
                    const i32 Y = static_cast<i32>(y_row[sx]);
                    const int32_t uv_offset = (sx >> 1) * uv_pixel_stride;
                    const i32 U = static_cast<i32>(u_row[uv_offset]) - 128;
                    const i32 V = static_cast<i32>(v_row[uv_offset]) - 128;

                    const i32 r = Y + ((359 * V) >> 8);
                    const i32 g = Y - ((88 * U + 183 * V) >> 8);
                    const i32 b = Y + ((454 * U) >> 8);

                    u32 dx = 0;
                    u32 dy = 0;
                    if (rot == 90) {
                        dx = static_cast<u32>(src_h - 1 - sy);
                        dy = static_cast<u32>(sx);
                    } else if (rot == 270) {
                        dx = static_cast<u32>(sy);
                        dy = static_cast<u32>(src_w - 1 - sx);
                    } else if (rot == 180) {
                        dx = static_cast<u32>(src_w - 1 - sx);
                        dy = static_cast<u32>(src_h - 1 - sy);
                    } else {
                        dx = static_cast<u32>(sx);
                        dy = static_cast<u32>(sy);
                    }

                    if (mirror_x) {
                        dx = dst_w - 1u - dx;
                    }

                    const usize out_idx = (static_cast<usize>(dy) * dst_w + dx) * 3u;
                    rgb[out_idx + 0] = clamp_byte(r);
                    rgb[out_idx + 1] = clamp_byte(g);
                    rgb[out_idx + 2] = clamp_byte(b);
                }
            }

            {
                std::lock_guard<std::mutex> lock(self->frame_mutex);
                self->latest_rgb = std::move(rgb);
                self->latest_width = dst_w;
                self->latest_height = dst_h;
                self->has_new_frame.store(true, std::memory_order_release);
            }
        }

        AImage_delete(image);
    }
};
#else
struct CameraService::AndroidCameraState {};
#endif

CameraService::CameraService() = default;

CameraService::~CameraService() {
    shutdown();
}

bool CameraService::initialize() {
    if (m_initialized) return true;

    m_devices.clear();

#if defined(__ANDROID__)
    m_android_cam = new AndroidCameraState();
    m_android_cam->manager = ACameraManager_create();
    if (m_android_cam->manager) {
        ACameraIdList* id_list = nullptr;
        if (ACameraManager_getCameraIdList(m_android_cam->manager, &id_list) == ACAMERA_OK && id_list) {
            for (int i = 0; i < id_list->numCameras; ++i) {
                const char* id_str = id_list->cameraIds[i];
                if (!id_str) continue;

                ACameraMetadata* chars = nullptr;
                if (ACameraManager_getCameraCharacteristics(m_android_cam->manager, id_str, &chars) != ACAMERA_OK || !chars) {
                    continue;
                }

                CameraDeviceInfo info{};
                info.device_index = static_cast<u32>(m_devices.size());
                info.android_camera_id = InplaceString<64>(StringRef(id_str));
                info.preferred_width = 640;
                info.preferred_height = 480;
                info.preferred_fps = 30.0f;
                info.is_synthetic = false;

                ACameraMetadata_const_entry entry{};
                if (ACameraMetadata_getConstEntry(chars, ACAMERA_LENS_FACING, &entry) == ACAMERA_OK && entry.count > 0) {
                    const uint8_t facing = entry.data.u8[0];
                    if (facing == ACAMERA_LENS_FACING_FRONT) {
                        info.facing = CameraFacing::Front;
                        info.name = InplaceString<64>("Front Selfie Camera");
                    } else if (facing == ACAMERA_LENS_FACING_EXTERNAL) {
                        info.facing = CameraFacing::External;
                        info.name = InplaceString<64>("External USB Camera");
                    } else {
                        info.facing = CameraFacing::Back;
                        info.name = InplaceString<64>("Rear Main Camera");
                    }
                } else {
                    info.name = InplaceString<64>("Android Camera");
                }

                if (ACameraMetadata_getConstEntry(chars, ACAMERA_SENSOR_ORIENTATION, &entry) == ACAMERA_OK && entry.count > 0) {
                    info.sensor_orientation_deg = entry.data.i32[0];
                }

                if (ACameraMetadata_getConstEntry(chars, ACAMERA_FLASH_INFO_AVAILABLE, &entry) == ACAMERA_OK && entry.count > 0) {
                    info.has_flash_unit = (entry.data.u8[0] != 0);
                }

                ACameraMetadata_free(chars);

                bool already_have_facing = false;
                for (const auto& existing : m_devices) {
                    if (existing.facing == info.facing) {
                        already_have_facing = true;
                        break;
                    }
                }
                if (!already_have_facing) {
                    m_devices.push_back(info);
                }
            }
            ACameraManager_deleteCameraIdList(id_list);
        }
    }
#endif

    // Always append Rear & Front synthetic cameras if not already present so desktop & unit tests work seamlessly
    if (m_devices.empty()) {
        CameraDeviceInfo back_sim{};
        back_sim.device_index = 0;
        back_sim.name = InplaceString<64>("Optical Test Camera (Rear 1x)");
        back_sim.facing = CameraFacing::Back;
        back_sim.sensor_orientation_deg = 90;
        back_sim.has_flash_unit = true;
        back_sim.preferred_width = 360;
        back_sim.preferred_height = 480;
        back_sim.preferred_fps = 60.0f;
        back_sim.is_synthetic = true;
        m_devices.push_back(back_sim);

        CameraDeviceInfo front_sim{};
        front_sim.device_index = 1;
        front_sim.name = InplaceString<64>("Optical Test Camera (Front Selfie)");
        front_sim.facing = CameraFacing::Front;
        front_sim.sensor_orientation_deg = 270;
        front_sim.has_flash_unit = false;
        front_sim.preferred_width = 360;
        front_sim.preferred_height = 480;
        front_sim.preferred_fps = 60.0f;
        front_sim.is_synthetic = true;
        m_devices.push_back(front_sim);
    }

    m_initialized = true;
    open_camera(0, m_devices[0].preferred_width, m_devices[0].preferred_height);
    return true;
}

void CameraService::shutdown() {
    close_camera();
#if defined(__ANDROID__)
    if (m_android_cam) {
        if (m_android_cam->manager) {
            ACameraManager_delete(m_android_cam->manager);
            m_android_cam->manager = nullptr;
        }
        delete m_android_cam;
        m_android_cam = nullptr;
    }
#endif
    m_devices.clear();
    m_initialized = false;
}

void CameraService::register_external_device(const CameraDeviceInfo& dev) {
    // Replace synthetic devices if registering first real external hardware device
    if (!m_devices.empty() && m_devices[0].is_synthetic) {
        m_devices.clear();
    }
    CameraDeviceInfo copy = dev;
    copy.device_index = static_cast<u32>(m_devices.size());
    m_devices.push_back(copy);
}

void CameraService::push_external_rgb_frame(ArrayRef<u8> rgb, u32 width, u32 height, usize row_stride) {
    if (width == 0 || height == 0 || rgb.empty()) return;

    const usize packed_row = static_cast<usize>(width) * 3u;
    const usize stride = (row_stride >= packed_row) ? row_stride : packed_row;
    if (rgb.size() < stride * static_cast<usize>(height - 1u) + packed_row) return;

    m_frame_width = width;
    m_frame_height = height;
    const usize total_bytes = packed_row * static_cast<usize>(height);
    if (m_rgb_pixels.size() != total_bytes) {
        m_rgb_pixels.resize(total_bytes);
    }

    if (stride == packed_row) {
        std::memcpy(m_rgb_pixels.data(), rgb.data(), total_bytes);
    } else {
        for (u32 y = 0; y < height; ++y) {
            std::memcpy(m_rgb_pixels.data() + static_cast<usize>(y) * packed_row,
                        rgb.data() + static_cast<usize>(y) * stride,
                        packed_row);
        }
    }

    m_using_synthetic = false;
    m_got_external_frame = true;
    m_fps_frame_count++;
    compute_frame_histograms();
}

bool CameraService::open_android_camera(const CameraDeviceInfo& dev, i32 width, i32 height) {
#if defined(__ANDROID__)
    if (!m_android_cam || !m_android_cam->manager || dev.android_camera_id.empty()) {
        return false;
    }

    close_android_camera();

    m_android_cam->sensor_orientation_deg = dev.sensor_orientation_deg;
    m_android_cam->facing = dev.facing;
    m_android_cam->camera_id = dev.android_camera_id;
    m_android_cam->torch_supported = dev.has_flash_unit;
    m_android_cam->device_disconnected.store(false, std::memory_order_release);
    m_android_cam->has_new_frame.store(false, std::memory_order_release);

    const i32 req_w = std::max(width, height);
    const i32 req_h = std::min(width, height);

    media_status_t mstatus = AImageReader_new(req_w, req_h, AIMAGE_FORMAT_YUV_420_888, 3, &m_android_cam->image_reader);
    if (mstatus != AMEDIA_OK || !m_android_cam->image_reader) {
        KEEL_CAM_LOGE("AImageReader_new(%d, %d) failed: %d", req_w, req_h, static_cast<int>(mstatus));
        close_android_camera();
        return false;
    }

    AImageReader_ImageListener img_listener{};
    img_listener.context = m_android_cam;
    img_listener.onImageAvailable = &AndroidCameraState::on_image_available;
    AImageReader_setImageListener(m_android_cam->image_reader, &img_listener);

    if (AImageReader_getWindow(m_android_cam->image_reader, &m_android_cam->reader_window) != AMEDIA_OK ||
        !m_android_cam->reader_window) {
        close_android_camera();
        return false;
    }

    ACameraDevice_StateCallbacks dev_callbacks{};
    dev_callbacks.context = m_android_cam;
    dev_callbacks.onDisconnected = &AndroidCameraState::on_device_disconnected;
    dev_callbacks.onError = &AndroidCameraState::on_device_error;

    camera_status_t cstatus = ACameraManager_openCamera(
        m_android_cam->manager, dev.android_camera_id.c_str(), &dev_callbacks, &m_android_cam->device);
    if (cstatus != ACAMERA_OK || !m_android_cam->device) {
        KEEL_CAM_LOGW("ACameraManager_openCamera(%s) failed: %d",
                      dev.android_camera_id.c_str(), static_cast<int>(cstatus));
        close_android_camera();
        return false;
    }

    if (ACameraDevice_createCaptureRequest(m_android_cam->device, TEMPLATE_PREVIEW, &m_android_cam->capture_request) != ACAMERA_OK) {
        close_android_camera();
        return false;
    }

    if (ACameraOutputTarget_create(m_android_cam->reader_window, &m_android_cam->output_target) != ACAMERA_OK) {
        close_android_camera();
        return false;
    }
    ACaptureRequest_addTarget(m_android_cam->capture_request, m_android_cam->output_target);

    if (ACaptureSessionOutputContainer_create(&m_android_cam->session_outputs) != ACAMERA_OK) {
        close_android_camera();
        return false;
    }
    if (ACaptureSessionOutput_create(m_android_cam->reader_window, &m_android_cam->session_output) != ACAMERA_OK) {
        close_android_camera();
        return false;
    }
    ACaptureSessionOutputContainer_add(m_android_cam->session_outputs, m_android_cam->session_output);

    ACameraCaptureSession_stateCallbacks session_callbacks{};
    session_callbacks.context = m_android_cam;
    session_callbacks.onClosed = &AndroidCameraState::on_session_closed;
    session_callbacks.onReady = &AndroidCameraState::on_session_ready;
    session_callbacks.onActive = &AndroidCameraState::on_session_active;

    if (ACameraDevice_createCaptureSession(
            m_android_cam->device, m_android_cam->session_outputs, &session_callbacks, &m_android_cam->capture_session) != ACAMERA_OK) {
        close_android_camera();
        return false;
    }

    if (ACameraCaptureSession_setRepeatingRequest(
            m_android_cam->capture_session, nullptr, 1, &m_android_cam->capture_request, nullptr) != ACAMERA_OK) {
        close_android_camera();
        return false;
    }

    KEEL_CAM_LOGI("Camera2 streaming started on camera %s (%dx%d)", dev.android_camera_id.c_str(), req_w, req_h);
    return true;
#else
    (void)dev;
    (void)width;
    (void)height;
    return false;
#endif
}

void CameraService::close_android_camera() {
#if defined(__ANDROID__)
    if (!m_android_cam) return;

    if (m_android_cam->capture_session) {
        ACameraCaptureSession_stopRepeating(m_android_cam->capture_session);
        ACameraCaptureSession_close(m_android_cam->capture_session);
        m_android_cam->capture_session = nullptr;
    }
    if (m_android_cam->capture_request) {
        if (m_android_cam->output_target) {
            ACaptureRequest_removeTarget(m_android_cam->capture_request, m_android_cam->output_target);
        }
        ACaptureRequest_free(m_android_cam->capture_request);
        m_android_cam->capture_request = nullptr;
    }
    if (m_android_cam->output_target) {
        ACameraOutputTarget_free(m_android_cam->output_target);
        m_android_cam->output_target = nullptr;
    }
    if (m_android_cam->session_output) {
        if (m_android_cam->session_outputs) {
            ACaptureSessionOutputContainer_remove(m_android_cam->session_outputs, m_android_cam->session_output);
        }
        ACaptureSessionOutput_free(m_android_cam->session_output);
        m_android_cam->session_output = nullptr;
    }
    if (m_android_cam->session_outputs) {
        ACaptureSessionOutputContainer_free(m_android_cam->session_outputs);
        m_android_cam->session_outputs = nullptr;
    }
    if (m_android_cam->device) {
        ACameraDevice_close(m_android_cam->device);
        m_android_cam->device = nullptr;
    }
    if (m_android_cam->image_reader) {
        AImageReader_delete(m_android_cam->image_reader);
        m_android_cam->image_reader = nullptr;
        m_android_cam->reader_window = nullptr;
    }
#endif
}

bool CameraService::open_camera(usize device_index, i32 width, i32 height) {
    if (!m_initialized) {
        initialize();
    }
    if (m_devices.empty()) return false;

    close_camera();

    if (device_index >= m_devices.size()) {
        device_index = 0;
    }
    m_active_device_index = device_index;

#if defined(__ANDROID__)
    const auto& dev = m_devices[device_index];
    if (!dev.is_synthetic) {
        if (open_android_camera(dev, width, height)) {
            m_using_synthetic = false;
            m_got_external_frame = false;
            m_frame_width = 360;
            m_frame_height = 480;
            m_rgb_pixels.resize(static_cast<usize>(m_frame_width) * static_cast<usize>(m_frame_height) * 3u, 0);
            m_streaming = true;
            m_paused = false;
            generate_procedural_frame(0.0);
            compute_frame_histograms();
            return true;
        }
    }
#endif

    m_using_synthetic = true;
    m_got_external_frame = false;
    m_frame_width = static_cast<u32>(std::clamp(width > 0 ? width : 360, 120, 1920));
    m_frame_height = static_cast<u32>(std::clamp(height > 0 ? height : 480, 120, 1920));
    if (m_frame_width > m_frame_height) {
        std::swap(m_frame_width, m_frame_height);
    }
    m_rgb_pixels.resize(static_cast<usize>(m_frame_width) * static_cast<usize>(m_frame_height) * 3u, 0);
    m_streaming = true;
    m_paused = false;
    generate_procedural_frame(0.0);
    compute_frame_histograms();
    return true;
}

void CameraService::close_camera() {
    close_android_camera();
    m_streaming = false;
    m_paused = false;
    m_got_external_frame = false;
}

void CameraService::set_paused(bool paused) {
    m_paused = paused;
}

bool CameraService::switch_camera_facing() {
    if (m_devices.size() <= 1) return false;
    const usize next_idx = (m_active_device_index + 1u) % m_devices.size();
    return open_camera(next_idx, static_cast<i32>(m_frame_width), static_cast<i32>(m_frame_height));
}

const CameraDeviceInfo* CameraService::active_device() const noexcept {
    if (m_active_device_index < m_devices.size()) {
        return &m_devices[m_active_device_index];
    }
    return nullptr;
}

void CameraService::set_flash_mode(FlashMode mode) {
    m_flash_mode = mode;
    set_torch_enabled(mode == FlashMode::Torch);
}

void CameraService::set_torch_enabled(bool enabled) {
    m_torch_enabled = enabled;
#if defined(__ANDROID__)
    if (m_android_cam && m_android_cam->capture_request && m_android_cam->capture_session &&
        m_android_cam->torch_supported) {
        uint8_t flash_mode = enabled ? ACAMERA_FLASH_MODE_TORCH : ACAMERA_FLASH_MODE_OFF;
        ACaptureRequest_setEntry_u8(m_android_cam->capture_request, ACAMERA_FLASH_MODE, 1, &flash_mode);
        ACameraCaptureSession_setRepeatingRequest(
            m_android_cam->capture_session, nullptr, 1, &m_android_cam->capture_request, nullptr);
    }
#endif
}

void CameraService::trigger_tap_to_focus(Vec2f normalized_point) noexcept {
    m_focus_point.x = std::clamp(normalized_point.x, 0.05f, 0.95f);
    m_focus_point.y = std::clamp(normalized_point.y, 0.05f, 0.95f);
}

void CameraService::set_exposure_compensation_ev(f32 ev) noexcept {
    m_exposure_ev = std::clamp(ev, -3.0f, 3.0f);
#if defined(__ANDROID__)
    if (m_android_cam && m_android_cam->capture_request && m_android_cam->capture_session) {
        int32_t ev_steps = static_cast<int32_t>(std::round(m_exposure_ev * 2.0f));
        ACaptureRequest_setEntry_i32(
            m_android_cam->capture_request, ACAMERA_CONTROL_AE_EXPOSURE_COMPENSATION, 1, &ev_steps);
        ACameraCaptureSession_setRepeatingRequest(
            m_android_cam->capture_session, nullptr, 1, &m_android_cam->capture_request, nullptr);
    }
#endif
}

void CameraService::set_zoom_factor(f32 zoom) noexcept {
    m_zoom_factor = std::clamp(zoom, 1.0f, 8.0f);
}

bool CameraService::capture_snapshot() {
    if (m_rgb_pixels.empty()) return false;
    m_last_captured_rgb.resize(m_rgb_pixels.size());
    std::memcpy(m_last_captured_rgb.data(), m_rgb_pixels.data(), m_rgb_pixels.size());
    m_captured_photos++;
    return true;
}

void CameraService::update_frame(f64 dt_seconds) {
    if (!m_streaming || m_paused) return;

    const f64 clamped_dt = std::clamp(dt_seconds, 0.0, 0.1);
    m_fps_accum_sec += clamped_dt;

    bool new_frame = false;

#if defined(__ANDROID__)
    if (m_android_cam && m_using_synthetic && m_active_device_index < m_devices.size() &&
        !m_devices[m_active_device_index].is_synthetic) {
        m_retry_timer_sec += clamped_dt;
        if (m_retry_timer_sec >= 1.5) {
            m_retry_timer_sec = 0.0;
            const auto& dev = m_devices[m_active_device_index];
            if (open_android_camera(dev, dev.preferred_width, dev.preferred_height)) {
                m_using_synthetic = false;
            }
        }
    }

    if (m_android_cam && !m_using_synthetic) {
        if (m_android_cam->device_disconnected.load(std::memory_order_acquire)) {
            close_android_camera();
            m_using_synthetic = true;
        } else if (m_android_cam->has_new_frame.exchange(false, std::memory_order_acq_rel)) {
            std::lock_guard<std::mutex> lock(m_android_cam->frame_mutex);
            if (!m_android_cam->latest_rgb.empty() &&
                m_android_cam->latest_width > 0 &&
                m_android_cam->latest_height > 0) {
                m_frame_width = m_android_cam->latest_width;
                m_frame_height = m_android_cam->latest_height;
                m_rgb_pixels = std::move(m_android_cam->latest_rgb);
                new_frame = true;
                m_fps_frame_count++;
            }
        }
    }
#endif

    if (m_using_synthetic && !m_got_external_frame) {
        generate_procedural_frame(clamped_dt);
        new_frame = true;
        m_fps_frame_count++;
    }

    if (m_fps_accum_sec >= 0.5) {
        m_stats.fps = static_cast<f32>(static_cast<f64>(m_fps_frame_count) / m_fps_accum_sec);
        m_fps_accum_sec = 0.0;
        m_fps_frame_count = 0;
    }

    if (new_frame) {
        compute_frame_histograms();
    }
}

void CameraService::generate_procedural_frame(f64 dt_seconds) {
    m_sim_time_sec += dt_seconds;
    const u32 w = m_frame_width;
    const u32 h = m_frame_height;
    if (w == 0 || h == 0) return;

    const usize total_bytes = static_cast<usize>(w) * static_cast<usize>(h) * 3u;
    if (m_rgb_pixels.size() != total_bytes) {
        m_rgb_pixels.resize(total_bytes);
    }

    const bool is_front = (active_device() && active_device()->facing == CameraFacing::Front);
    const f32 ev_gain = std::pow(2.0f, m_exposure_ev * 0.5f) * (m_torch_enabled ? 1.25f : 1.0f);
    const f32 zoom = m_zoom_factor;
    const f32 t = static_cast<f32>(m_sim_time_sec);

    const f32 target_cx = 0.5f + 0.22f * std::cos(t * 0.9f) * (is_front ? -1.0f : 1.0f);
    const f32 target_cy = 0.46f + 0.16f * std::sin(t * 1.3f);

    static constexpr u8 kBars[7][3] = {
        {215, 215, 215}, // White
        {215, 215,  24}, // Yellow
        { 24, 215, 215}, // Cyan
        { 24, 215,  24}, // Green
        {215,  24, 215}, // Magenta
        {215,  24,  24}, // Red
        { 24,  24, 215}  // Blue
    };

    const f32 inv_w = 1.0f / static_cast<f32>(w);
    const f32 inv_h = 1.0f / static_cast<f32>(h);
    const f32 aspect = static_cast<f32>(w) / static_cast<f32>(h);

    for (u32 y = 0; y < h; ++y) {
        const f32 v_raw = (static_cast<f32>(y) + 0.5f) * inv_h;
        const f32 v = (v_raw - m_focus_point.y) / zoom + m_focus_point.y;
        u8* row_ptr = m_rgb_pixels.data() + static_cast<usize>(y) * static_cast<usize>(w) * 3u;

        for (u32 x = 0; x < w; ++x) {
            const f32 u_raw = (static_cast<f32>(x) + 0.5f) * inv_w;
            const f32 u = (u_raw - m_focus_point.x) / zoom + m_focus_point.x;

            f32 r = 24.0f;
            f32 g = 28.0f;
            f32 b = 36.0f;

            if (v < 0.20f) {
                const i32 bar_idx = std::clamp(static_cast<i32>(u * 7.0f), 0, 6);
                const i32 idx = is_front ? (6 - bar_idx) : bar_idx;
                r = static_cast<f32>(kBars[idx][0]);
                g = static_cast<f32>(kBars[idx][1]);
                b = static_cast<f32>(kBars[idx][2]);
            } else if (v > 0.85f) {
                const f32 luma = std::clamp(u * 255.0f, 0.0f, 255.0f);
                r = luma;
                g = luma;
                b = luma;
            } else {
                const f32 sky = 1.0f - (v - 0.20f) / 0.65f;
                r = 26.0f + 38.0f * sky;
                g = 34.0f + 56.0f * sky;
                b = 48.0f + 85.0f * sky;

                const f32 gx = std::fmod(u * 12.0f + 100.0f, 1.0f);
                const f32 gy = std::fmod(v * 16.0f + 100.0f, 1.0f);
                if (gx < 0.035f || gy < 0.035f) {
                    r += 18.0f;
                    g += 24.0f;
                    b += 30.0f;
                }

                const f32 dx = (u - target_cx) * aspect;
                const f32 dy = (v - target_cy);
                const f32 dist = std::sqrt(dx * dx + dy * dy);
                if (dist < 0.16f) {
                    const f32 angle = std::atan2(dy, dx) + t * 0.6f;
                    const f32 spoke = std::cos(angle * 12.0f);
                    const f32 star_val = (spoke > 0.0f) ? 225.0f : 42.0f;
                    const f32 ring_edge = std::abs(dist - 0.155f);
                    if (ring_edge < 0.006f) {
                        r = 46.0f;
                        g = 215.0f;
                        b = 145.0f;
                    } else if (dist < 0.02f) {
                        r = 245.0f;
                        g = 90.0f;
                        b = 75.0f;
                    } else {
                        r = star_val * (is_front ? 0.88f : 1.0f);
                        g = star_val * 0.95f;
                        b = star_val;
                    }
                }
            }

            const f32 vig_dx = (u_raw - 0.5f) * 1.35f;
            const f32 vig_dy = (v_raw - 0.5f) * 1.35f;
            const f32 vig = std::clamp(1.0f - 0.28f * (vig_dx * vig_dx + vig_dy * vig_dy), 0.55f, 1.0f);

            row_ptr[x * 3u + 0] = clamp_byte_f32(r * vig * ev_gain);
            row_ptr[x * 3u + 1] = clamp_byte_f32(g * vig * ev_gain);
            row_ptr[x * 3u + 2] = clamp_byte_f32(b * vig * ev_gain);
        }
    }
}

void CameraService::compute_frame_histograms() {
    std::memset(m_stats.hist_luma, 0, sizeof(m_stats.hist_luma));
    std::memset(m_stats.hist_r, 0, sizeof(m_stats.hist_r));
    std::memset(m_stats.hist_g, 0, sizeof(m_stats.hist_g));
    std::memset(m_stats.hist_b, 0, sizeof(m_stats.hist_b));

    m_stats.width = m_frame_width;
    m_stats.height = m_frame_height;
    m_stats.frame_sequence++;

    const usize pixel_count = static_cast<usize>(m_frame_width) * static_cast<usize>(m_frame_height);
    if (pixel_count == 0 || m_rgb_pixels.size() < pixel_count * 3u) {
        m_rgba_pixels.clear();
        m_stats.mean_luminance = 0.0f;
        m_stats.contrast_rms = 0.0f;
        return;
    }

    if (m_rgba_pixels.size() != pixel_count) {
        m_rgba_pixels.resize(pixel_count);
    }

    const u8* rgb = m_rgb_pixels.data();
    u32* rgba = m_rgba_pixels.data();
    f64 sum_luma = 0.0;
    f64 sum_luma_sq = 0.0;

    for (usize i = 0; i < pixel_count; ++i) {
        const u8 r = rgb[i * 3u + 0];
        const u8 g = rgb[i * 3u + 1];
        const u8 b = rgb[i * 3u + 2];
        rgba[i] = static_cast<u32>(r) |
                  (static_cast<u32>(g) << 8) |
                  (static_cast<u32>(b) << 16) |
                  0xFF000000u;

        // ITU-R BT.709 integer approximation: Y = (54*R + 183*G + 19*B) >> 8
        const u32 y = (54u * r + 183u * g + 19u * b) >> 8u;
        const u8 luma = static_cast<u8>(std::min<u32>(y, 255u));

        m_stats.hist_r[r]++;
        m_stats.hist_g[g]++;
        m_stats.hist_b[b]++;
        m_stats.hist_luma[luma]++;

        sum_luma += static_cast<f64>(luma);
        sum_luma_sq += static_cast<f64>(luma) * static_cast<f64>(luma);
    }

    const f64 inv_n = 1.0 / static_cast<f64>(pixel_count);
    const f64 mean = sum_luma * inv_n;
    const f64 variance = std::max(0.0, sum_luma_sq * inv_n - mean * mean);
    m_stats.mean_luminance = static_cast<f32>(mean);
    m_stats.contrast_rms = static_cast<f32>(std::sqrt(variance));
}

} // namespace keel
