import AVFoundation
import SceneKit
import UIKit

class XRSLAMer : CameraDelegate, MotionDelegate {

    let xrslam: XRSLAM
    weak var targetView: UIImageView? = nil

    var position: SCNVector3
    var rotation: SCNQuaternion
    var fov: Double

    var virtualObjectIds: [size_t] = []
    var virtualObjectPositions: [SCNVector3] = []
    var virtualObjectRotations: [SCNQuaternion] = []
    var virtualObjectIsolated: [Int] = []
    var stopFlag = true

    init() {
        xrslam = XRSLAM(UIDevice().type.rawValue)
        position = SCNVector3Make(0, 0, 0)
        rotation = SCNVector4Make(0, 0, 0, 1)
        fov = xrslam.getFOV()
    }

    func cameraDidOutput(timestamp: CMTime, sampleBuffer: CMSampleBuffer) {
        if self.stopFlag{
            xrslam.processBuffer(sampleBuffer)
        }
        else{
            xrslam.trackCamera(timestamp.seconds, buffer: sampleBuffer)
        }
    }

    func motionDidGyroscopeUpdate(timestamp: Double, rotationRateX: Double, rotationRateY: Double, rotationRateZ: Double) {
        if self.stopFlag{
            return;
        }
        xrslam.trackGyroscope(timestamp, x: rotationRateX, y: rotationRateY, z: rotationRateZ)
    }

    func motionDidAccelerometerUpdate(timestamp: Double, accelerationX: Double, accelerationY: Double, accelerationZ: Double) {
        if self.stopFlag{
            return;
        }
        xrslam.trackAccelerometer(timestamp, x: accelerationX, y: accelerationY, z: accelerationZ)
    }

    func getCameraPosition() -> SCNVector3 {
        return xrslam.getCameraPosition()
    }

    func getCameraRotation() -> SCNQuaternion {
        return xrslam.getCameraRotation()
    }

    func getUIImage() -> UIImage {
        if self.stopFlag{
            return xrslam.getCurrentImage()
        }else{
            return xrslam.getLatestImage()
        }
    }

    func createVirtualObject() -> Bool {
        let id = xrslam.createVirtualObject()
        xrslam.updateVirtualObject(id)
        if id != size_t(-1) {
            virtualObjectIds.append(id)
            virtualObjectPositions.append(xrslam.getVirtualObjectPosition())
            virtualObjectRotations.append(xrslam.getVirtualObjectRotation())
            virtualObjectIsolated.append(0)
            return true
        } else {
            return false
        }
    }

    func updateVirtualObjectPoses() {
        for (i, id) in virtualObjectIds.enumerated() {
            xrslam.updateVirtualObject(id)
            virtualObjectPositions[i] = xrslam.getVirtualObjectPosition()
            virtualObjectRotations[i] = xrslam.getVirtualObjectRotation()
            virtualObjectIsolated[i] = Int(xrslam.getVirtualObjectIsolated())
        }
    }

    func getSystemState() -> size_t {
        let state = xrslam.get_system_state().rawValue
        return size_t(state)
    }

    func Stop() {
        stopFlag = true
    }

    func Begin() {
        stopFlag = false
    }

    func isRuning() -> Bool {
        return !stopFlag;
    }

    func cameraDidDrop() {
        debugPrint("[XRSLAM.ios][warning] frame dropped!")
    }

    func EnableLoc() {
        xrslam.enable_global_localization()
    }

    func DisableLoc() {
        xrslam.disable_global_localization()
    }

    func query_frame() {
        xrslam.query_frame()
    }

    func global_localization_initialized() -> Int32 {
        return xrslam.global_localization_initialized()
    }

    func get_logger_message() -> NSMutableArray {
        return xrslam.get_logger_message()
    }
}
