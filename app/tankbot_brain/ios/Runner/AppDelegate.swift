import Flutter
import UIKit
import ARKit

@main
@objc class AppDelegate: FlutterAppDelegate {
  private var arPose: ArkitPoseStreamer?

  override func application(
    _ application: UIApplication,
    didFinishLaunchingWithOptions launchOptions: [UIApplication.LaunchOptionsKey: Any]?
  ) -> Bool {
    GeneratedPluginRegistrant.register(with: self)
    if let registrar = self.registrar(forPlugin: "ArkitPoseStreamer") {
      arPose = ArkitPoseStreamer(messenger: registrar.messenger())
    }
    return super.application(application, didFinishLaunchingWithOptions: launchOptions)
  }
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
      default:
        result(FlutterMethodNotImplemented)
      }
    }
  }

  private func run(reset: Bool) {
    guard ARWorldTrackingConfiguration.isSupported else { return }
    let config = ARWorldTrackingConfiguration()
    config.worldAlignment = .gravity
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

  func session(_ session: ARSession, didUpdate frame: ARFrame) {
    guard let sink = sink else { return }
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
      "t": frame.timestamp,
      "sent": ProcessInfo.processInfo.systemUptime,
      "x": Double(pos.x), "y": Double(pos.y), "z": Double(pos.z),
      "fx": Double(-back.x), "fy": Double(-back.y), "fz": Double(-back.z),
      "state": state,
    ]
    DispatchQueue.main.async { sink(msg) }
  }
}
