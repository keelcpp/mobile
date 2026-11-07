# `keel::mobile` — Cross-Platform Mobile & Embedded Hardware Abstraction Layer

`keel::mobile` is a lightweight, zero-STL-header C++20 hardware abstraction library built on top of [`keel`](https://github.com/keelcpp/keel) for mobile, embedded, and desktop-simulated applications:

- **9-DOF IMU & Environmental Sensors (`keel::SensorManager`)**:
  - Accelerometer ($\text{m/s}^2$), Linear Acceleration, Gravity Vector, 3-Axis Gyroscope ($\text{rad/s}$), 3-Axis Magnetometer ($\mu\text{T}$), Barometric Pressure ($\text{hPa}$) + ISA Barometric Altitude ($\text{m}$), Ambient Light ($\text{lux}$), Proximity ($\text{cm}$), and Pedometer Step Counter.
  - Quaternion & Euler attitude estimation (`FlatTable` and `Handheld` reference frames) with one-tap orientation tare (`tare_orientation()`) and zero-allocation history buffers.
  - Native Android NDK `ASensorManager` / `ALooper` backend + deterministic 6-DOF desktop physics simulator.
- **GNSS Location & Satellite Skyplot (`keel::LocationService`)**:
  - WGS-84 position fixes (`GeoCoord`), altitude, ground speed, course bearing, horizontal/vertical accuracy, Haversine great-circle distance (`distance_meters`), initial forward azimuth (`bearing_degrees`), breadcrumb route history, and 12-channel multi-constellation (GPS / GLONASS / Galileo / BeiDou) skyplot telemetry.
  - Native Android JNI `LocationManager` backend + figure-8 test route simulator.
- **Real-Time Audio Capture & 1024-Point FFT (`keel::AudioCapture`)**:
  - Lock-free PCM capture, RMS / Peak / dBFS level metering, Hann-windowed 1024-point Radix-2 Cooley-Tukey FFT (`kSpectrumBins = 512`), exponential spectral smoothing, peak-hold envelope, dominant frequency detection, and multi-harmonic + white-noise signal synthesis.
- **Mobile Camera & Real-Time Histograms (`keel::CameraService`)**:
  - Native Android Camera2 NDK (`ACameraManager` / `AImageReader`) low-latency `YUV_420_888` to `RGB888`/`RGBA8888` conversion with sensor orientation rotation and selfie mirroring, torch/flashlight control, EV exposure compensation, tap-to-focus, digital zoom, external frame injection, procedural SMPTE/Siemens optical test camera, and real-time 256-bin R/G/B/Luminance histograms.
- **Mobile OS System Bridge (`keel::SystemBridge`)**:
  - Runtime permissions (`Camera`, `RecordAudio`, `FineLocation`, `BodySensors`, `Bluetooth`, `Notifications`), tactile haptics (`HapticStyle` + Android JNI `Vibrator`), display safe-area insets (`SafeAreaInsets`), and battery/thermal telemetry.

## Building & Running Tests

```bash
cmake -B build
cmake --build build
ctest --test-dir build --output-on-failure
```
