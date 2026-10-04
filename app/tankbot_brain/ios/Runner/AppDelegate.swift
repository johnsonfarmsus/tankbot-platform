import Flutter
import UIKit
import ARKit
import CoreMotion
import CoreLocation

@main
@objc class AppDelegate: FlutterAppDelegate {
  private var arPose: ArkitPoseStreamer?
  private var location: LocationStreamer?

  override func application(
    _ application: UIApplication,
    didFinishLaunchingWithOptions launchOptions: [UIApplication.LaunchOptionsKey: Any]?
  ) -> Bool {
    GeneratedPluginRegistrant.register(with: self)
    if let registrar = self.registrar(forPlugin: "ArkitPoseStreamer") {
      arPose = ArkitPoseStreamer(messenger: registrar.messenger())
      location = LocationStreamer(messenger: registrar.messenger())
    }
    return super.application(application, didFinishLaunchingWithOptions: launchOptions)
  }
}

/// Streams GPS fixes and compass headings to Flutter, for logging and evaluation.
/// Event channel "tankbot/location": maps with type
///   "gps"     t (unix s), lat, lon, hAcc (m), alt, vAcc, speed, course
///   "heading" t, mag, true (degrees), acc (degrees), x, y, z (raw field, microtesla)
///   "auth"    status, precise
final class LocationStreamer: NSObject, FlutterStreamHandler, CLLocationManagerDelegate {
  private let mgr = CLLocationManager()
  private var sink: FlutterEventSink?

  init(messenger: FlutterBinaryMessenger) {
    super.init()
    mgr.delegate = self
    mgr.desiredAccuracy = kCLLocationAccuracyBest
    mgr.distanceFilter = kCLDistanceFilterNone
    mgr.headingFilter = 1
    mgr.headingOrientation = .portrait
    FlutterEventChannel(name: "tankbot/location", binaryMessenger: messenger).setStreamHandler(self)
  }

  func onListen(withArguments arguments: Any?, eventSink events: @escaping FlutterEventSink) -> FlutterError? {
    sink = events
    mgr.requestWhenInUseAuthorization()
    mgr.startUpdatingLocation()
    if CLLocationManager.headingAvailable() { mgr.startUpdatingHeading() }
    return nil
  }

  func onCancel(withArguments arguments: Any?) -> FlutterError? {
    mgr.stopUpdatingLocation()
    mgr.stopUpdatingHeading()
    sink = nil
    return nil
  }

  func locationManagerDidChangeAuthorization(_ manager: CLLocationManager) {
    if #available(iOS 14.0, *) {
      sink?(["type": "auth", "status": Int(manager.authorizationStatus.rawValue), "precise": manager.accuracyAuthorization == .fullAccuracy])
    }
    if sink != nil { manager.startUpdatingLocation() }
  }

  func locationManager(_ manager: CLLocationManager, didUpdateLocations locations: [CLLocation]) {
    for l in locations {
      sink?(["type": "gps", "t": l.timestamp.timeIntervalSince1970, "lat": l.coordinate.latitude, "lon": l.coordinate.longitude,
             "hAcc": l.horizontalAccuracy, "alt": l.altitude, "vAcc": l.verticalAccuracy, "speed": l.speed, "course": l.course])
    }
  }

  func locationManager(_ manager: CLLocationManager, didUpdateHeading h: CLHeading) {
    sink?(["type": "heading", "t": h.timestamp.timeIntervalSince1970, "mag": h.magneticHeading, "true": h.trueHeading,
           "acc": h.headingAccuracy, "x": h.x, "y": h.y, "z": h.z])
  }

  func locationManager(_ manager: CLLocationManager, didFailWithError error: Error) {
    sink?(["type": "error", "msg": error.localizedDescription])
  }

  func locationManagerShouldDisplayHeadingCalibration(_ manager: CLLocationManager) -> Bool { false }
}

/// Streams the phone's ARKit pose to Flutter at ~30 Hz.
/// Event channel "tankbot/arkit_pose": maps with
///   t      ARFrame timestamp (s, system uptime clock)
///   sent   system uptime when the event was created (s), for clock alignment in Dart
///   x,y,z  camera position (m), ARKit world frame (y up, gravity aligned)
///   fx,fy,fz  camera forward vector (the back camera's viewing direction)
///   state  "normal", "notAvailable" or "limited:<reason>"
/// Method channel "tankbot/arkit": "supported" -> Bool, "reset" -> restart tracking from zero.
final class ArkitPoseStreamer: NSObject, FlutterStreamHandler, ARSessionDelegate {
  private let session = ARSession()
  private var sink: FlutterEventSink?
  private var lastSent: TimeInterval = 0
  private var depthCounter = 0
  var depthEnabled = true

  init(messenger: FlutterBinaryMessenger) {
    super.init()
    session.delegate = self
    session.delegateQueue = DispatchQueue(label: "tankbot.arkit.pose")
    FlutterEventChannel(name: "tankbot/arkit_pose", binaryMessenger: messenger).setStreamHandler(self)
    FlutterMethodChannel(name: "tankbot/arkit", binaryMessenger: messenger).setMethodCallHandler { [weak self] call, result in
      switch call.method {
      case "supported":
        result(ARWorldTrackingConfiguration.isSupported)
      case "reset":
        self?.run(reset: true)
        result(nil)
      case "capabilities":
        var sys = utsname()
        uname(&sys)
        let model = withUnsafePointer(to: &sys.machine) {
          $0.withMemoryRebound(to: CChar.self, capacity: 1) { String(cString: $0) }
        }
        let tracking = ARWorldTrackingConfiguration.isSupported
        var depth = false, mesh = false
        if #available(iOS 14.0, *) { depth = ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth) }
        if #available(iOS 13.4, *) { mesh = ARWorldTrackingConfiguration.supportsSceneReconstruction(.meshWithClassification) }
        result([
          "platform": "ios",
          "model": model,
          "worldTracking": tracking,
          "sceneDepth": depth,
          "meshClassification": mesh,
          "worldMaps": tracking,
          "barometer": CMAltimeter.isRelativeAltitudeAvailable(),
          "gyro": CMMotionManager().isGyroAvailable,
          "gps": UIDevice.current.userInterfaceIdiom == .phone,
        ])
      case "documentsDir":
        result(FileManager.default.urls(for: .documentDirectory, in: .userDomainMask).first?.path)
      case "keepAwake":
        let on = (call.arguments as? Bool) ?? true
        DispatchQueue.main.async { UIApplication.shared.isIdleTimerDisabled = on }
        result(nil)
      default:
        result(FlutterMethodNotImplemented)
      }
    }
  }

  private func run(reset: Bool) {
    guard ARWorldTrackingConfiguration.isSupported else { return }
    let config = ARWorldTrackingConfiguration()
    config.worldAlignment = .gravity
    if #available(iOS 14.0, *), depthEnabled, ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth) {
      config.frameSemantics.insert(.sceneDepth)
    }
    session.run(config, options: reset ? [.resetTracking, .removeExistingAnchors] : [])
  }

  func onListen(withArguments arguments: Any?, eventSink events: @escaping FlutterEventSink) -> FlutterError? {
    sink = events
    run(reset: false)
    return nil
  }

  func onCancel(withArguments arguments: Any?) -> FlutterError? {
    sink = nil
    session.pause()
    return nil
  }

  /// Depth image -> points relative to the camera in a level, heading-aligned frame (fwd, left, up).
  @available(iOS 14.0, *)
  private func emitDepth(_ frame: ARFrame, _ sink: @escaping FlutterEventSink) {
    guard let depth = frame.sceneDepth else { return }
    let map = depth.depthMap
    guard let confMap = depth.confidenceMap else { return }
    CVPixelBufferLockBaseAddress(map, .readOnly)
    CVPixelBufferLockBaseAddress(confMap, .readOnly)
    defer {
      CVPixelBufferUnlockBaseAddress(map, .readOnly)
      CVPixelBufferUnlockBaseAddress(confMap, .readOnly)
    }
    guard let dBase = CVPixelBufferGetBaseAddress(map), let cBase = CVPixelBufferGetBaseAddress(confMap) else { return }
    let w = CVPixelBufferGetWidth(map), h = CVPixelBufferGetHeight(map)
    let dRow = CVPixelBufferGetBytesPerRow(map) / 4, cRow = CVPixelBufferGetBytesPerRow(confMap)
    let dPtr = dBase.assumingMemoryBound(to: Float32.self)
    let cPtr = cBase.assumingMemoryBound(to: UInt8.self)
    let intr = frame.camera.intrinsics
    let sx = Float(w) / Float(frame.camera.imageResolution.width)
    let sy = Float(h) / Float(frame.camera.imageResolution.height)
    let fx = intr[0][0] * sx, fy = intr[1][1] * sy, cx = intr[2][0] * sx, cy = intr[2][1] * sy
    let T = frame.camera.transform
    let camPos = simd_make_float3(T.columns.3)
    let look = -simd_make_float3(T.columns.2)
    var f = simd_float3(look.x, 0, look.z)
    let fl = simd_length(f)
    if fl < 0.2 { return } // camera pointing straight up/down: no usable heading
    f /= fl
    let left = simd_float3(f.z, 0, -f.x)
    var out = [Float32]()
    out.reserveCapacity(4096 * 3)
    var v = 0
    while v < h {
      var u = 0
      while u < w {
        if cPtr[v * cRow + u] >= 2 {
          let z = dPtr[v * dRow + u]
          if z > 0.15 && z < 3.5 {
            let x = (Float(u) - cx) * z / fx, y = (Float(v) - cy) * z / fy
            let pw = T * simd_float4(x, -y, -z, 1)
            let rel = simd_make_float3(pw) - camPos
            out.append(simd_dot(rel, f)); out.append(simd_dot(rel, left)); out.append(rel.y)
          }
        }
        u += 4
      }
      v += 4
    }
    let data = out.withUnsafeBufferPointer { Data(buffer: $0) }
    let msg: [String: Any] = ["type": "depth", "t": frame.timestamp, "pts": FlutterStandardTypedData(float32: data)]
    DispatchQueue.main.async { sink(msg) }
  }

  func session(_ session: ARSession, didUpdate frame: ARFrame) {
    guard let sink = sink else { return }
    depthCounter += 1
    if #available(iOS 14.0, *), depthCounter % 6 == 0 { emitDepth(frame, sink) }
    if frame.timestamp - lastSent < 1.0 / 30.0 { return }
    lastSent = frame.timestamp
    let m = frame.camera.transform
    let pos = m.columns.3
    let back = m.columns.2 // the camera looks along its -Z axis
    let state: String
    switch frame.camera.trackingState {
    case .normal: state = "normal"
    case .notAvailable: state = "notAvailable"
    case .limited(let reason): state = "limited:\(reason)"
    }
    let msg: [String: Any] = [
      "type": "pose",
      "t": frame.timestamp,
      "sent": ProcessInfo.processInfo.systemUptime,
      "x": Double(pos.x), "y": Double(pos.y), "z": Double(pos.z),
      "fx": Double(-back.x), "fy": Double(-back.y), "fz": Double(-back.z),
      "state": state,
    ]
    DispatchQueue.main.async { sink(msg) }
  }
}
