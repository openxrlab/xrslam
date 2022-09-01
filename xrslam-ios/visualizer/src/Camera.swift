import AVFoundation

protocol CameraDelegate : class {
    func cameraDidOutput(timestamp: CMTime, sampleBuffer: CMSampleBuffer)
    func cameraDidDrop()
}

extension CameraDelegate {
    func cameraDidDrop() {
    }
}

class Camera : NSObject {
    weak var delegate: CameraDelegate? = nil

    var imageWidth: Int {
        get {
            return self.output.videoSettings?["Width"] as? Int ?? 0
        }
    }

    var imageHeight: Int {
        get {
            return self.output.videoSettings?["Height"] as? Int ?? 0
        }
    }

    init?(position: AVCaptureDevice.Position = .back, preset: AVCaptureSession.Preset = .vga640x480, queue: DispatchQueue? = nil) {
        guard let device = Camera.avCaptureDevice(position: position) else {
            return nil
        }
        guard let input = try? AVCaptureDeviceInput(device: device) else {
            return nil
        }

        self.device = device
        self.input = input
        self.output = AVCaptureVideoDataOutput()
        self.session = AVCaptureSession()
        self.sampleBufferDelegate = SampleBufferDelegate()

        super.init()

        self.sampleBufferDelegate.camera = self

        self.output.videoSettings = [kCVPixelBufferPixelFormatTypeKey: kCVPixelFormatType_32BGRA] as [String: AnyObject]
        self.output.setSampleBufferDelegate(self.sampleBufferDelegate, queue: queue ?? .main)

        self.session.addInput(self.input)
        self.session.addOutput(self.output)

        self.session.beginConfiguration()
        self.session.sessionPreset = preset
        self.session.commitConfiguration()

        self.session.startRunning()
    }

    deinit {
        self.session.stopRunning()
    }

    func createPreviewLayer() -> AVCaptureVideoPreviewLayer {
        return AVCaptureVideoPreviewLayer(session: self.session)
    }

    func setAutoFocus() {
        guard let _ = try? self.device.lockForConfiguration() else {
            return
        }
        self.device.focusMode = .continuousAutoFocus
        self.device.unlockForConfiguration()
    }

    func setFocus(_ value: Float) {
        guard let _ = try? self.device.lockForConfiguration() else {
            return
        }
        self.device.setFocusModeLocked(lensPosition: value, completionHandler: nil)
        self.device.unlockForConfiguration()
    }

    func setAutoExposure() {
        guard let _ = try? self.device.lockForConfiguration() else {
            return
        }
        self.device.exposureMode = .continuousAutoExposure
        self.device.unlockForConfiguration()
    }

    func setExposure(_ value: Float) {
        guard let _ = try? self.device.lockForConfiguration() else {
            return
        }
        let minValue = self.device.activeFormat.minExposureDuration.seconds
        let maxValue = self.device.activeFormat.maxExposureDuration.seconds
        let exposure = (maxValue - minValue) * Double(value) + minValue
        let duration = CMTime(seconds: exposure, preferredTimescale: 1000000000)
        self.device.setExposureModeCustom(duration: duration, iso: AVCaptureDevice.currentISO, completionHandler: nil)
        self.device.unlockForConfiguration()
    }

    func setIso(_ value: Float) {
        guard let _ = try? self.device.lockForConfiguration() else {
            return
        }
        let minValue = self.device.activeFormat.minISO
        let maxValue = self.device.activeFormat.maxISO
        let iso = (maxValue - minValue) * value + minValue
        self.device.setExposureModeCustom(duration: AVCaptureDevice.currentExposureDuration, iso: iso, completionHandler: nil)
        self.device.unlockForConfiguration()
    }

    func setFps(_ value: Float) {
        guard let _ = try? self.device.lockForConfiguration() else {
            return
        }
        let fps = Int32(value)
        let format = device.activeFormat
        for fpsRange in format.videoSupportedFrameRateRanges {
            if fpsRange.minFrameRate <= Double(fps) && fpsRange.maxFrameRate >= Double(fps) {
                device.activeVideoMinFrameDuration = CMTime(value: 1, timescale: fps)
                device.activeVideoMaxFrameDuration = CMTime(value: 1, timescale: fps)
                break
            }
        }
        device.unlockForConfiguration()
    }

    private class func avCaptureDevice(position: AVCaptureDevice.Position) -> AVCaptureDevice? {
        let discoverySession = AVCaptureDevice.DiscoverySession(deviceTypes: [.builtInWideAngleCamera], mediaType: .video, position: position)
        for device in discoverySession.devices {
            if device.position == position {
                return device
            }
        }
        return nil
    }

    private let device: AVCaptureDevice
    private let input: AVCaptureInput
    private let output: AVCaptureVideoDataOutput
    private let session: AVCaptureSession
    private let sampleBufferDelegate: SampleBufferDelegate

    private class SampleBufferDelegate : NSObject, AVCaptureVideoDataOutputSampleBufferDelegate {
        weak var camera: Camera? = nil

        func captureOutput(_ output: AVCaptureOutput, didOutput sampleBuffer: CMSampleBuffer, from connection: AVCaptureConnection) {
            let pts = sampleBuffer.presentationTimeStamp
            self.camera?.delegate?.cameraDidOutput(timestamp: pts, sampleBuffer: sampleBuffer)
        }

        func captureOutput(_ output: AVCaptureOutput, didDrop sampleBuffer: CMSampleBuffer, from connection: AVCaptureConnection) {
            self.camera?.delegate?.cameraDidDrop()
        }
    }
}
