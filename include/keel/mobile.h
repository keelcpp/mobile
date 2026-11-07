#pragma once

#include "keel/mobile/audio.h"
#include "keel/mobile/camera.h"
#include "keel/mobile/location.h"
#include "keel/mobile/sensors.h"
#include "keel/mobile/system.h"
#include "keel/mobile/types.h"

namespace keel::mobile {

using keel::Vec2f;
using keel::Vec3f;
using keel::Quaternion;
using keel::EulerAngles;
using keel::GeoCoord;
using keel::SafeAreaInsets;
using keel::ScreenOrientation;
using keel::Permission;
using keel::PermissionStatus;
using keel::HapticStyle;
using keel::BatteryStatus;
using keel::DeviceInfo;

using keel::SensorType;
using keel::AttitudeReference;
using keel::SensorRate;
using keel::SensorInfo;
using keel::SensorSnapshot;
using keel::SensorSimConfig;
using keel::SensorManager;

using keel::LocationFix;
using keel::LocationService;

using keel::AudioCapture;

using keel::CameraFacing;
using keel::FlashMode;
using keel::CameraDeviceInfo;
using keel::CameraFrameStats;
using keel::CameraService;

using keel::SystemBridge;

} // namespace keel::mobile
