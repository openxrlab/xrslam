import CoreMotion

fileprivate let GRAVITY_NOMINAL = -9.80665

protocol MotionDelegate : class {
    func motionDidGyroscopeUpdate(timestamp: Double, rotationRateX: Double, rotationRateY: Double, rotationRateZ: Double)
    func motionDidAccelerometerUpdate(timestamp: Double, accelerationX: Double, accelerationY: Double, accelerationZ: Double)
    func motionDidMagnetometerUpdate(timestamp: Double, magnetFieldX: Double, magnetFieldY: Double, magnetFieldZ: Double)
    func motionDidAltimeterUpdate(timestamp: Double, pressure: Double, relativeAltitude: Double)
    func motionDidDeviceMotionUpdate(deviceMotion: CMDeviceMotion)
}

extension MotionDelegate {
    func motionDidMagnetometerUpdate(timestamp: Double, magnetFieldX: Double, magnetFieldY: Double, magnetFieldZ: Double) {
    }

    func motionDidAltimeterUpdate(timestamp: Double, pressure: Double, relativeAltitude: Double) {
    }

    func motionDidDeviceMotionUpdate(deviceMotion: CMDeviceMotion) {
    }
}

class Motion: NSObject {
    weak var delegate: MotionDelegate? = nil

    init?(updateInterval: TimeInterval = 0.01, useMagnetometer: Bool = false, useDeviceMotion: Bool = false, useAltimeter: Bool = false, queue: OperationQueue? = nil) {
        guard updateInterval > 0 else {
            return nil
        }

        super.init()

        self.motionManager.gyroUpdateInterval = updateInterval
        self.motionManager.accelerometerUpdateInterval = updateInterval
        self.motionManager.magnetometerUpdateInterval = updateInterval
        self.motionManager.deviceMotionUpdateInterval = updateInterval

        let q = queue ?? .main

        if self.motionManager.isGyroAvailable {
            self.motionManager.startGyroUpdates(to: q) {
                [weak self] (data, error) in
                guard let delegate = self?.delegate, let record = data, error == nil else {
                    return
                }
                delegate.motionDidGyroscopeUpdate(timestamp: record.timestamp, rotationRateX: record.rotationRate.x, rotationRateY: record.rotationRate.y, rotationRateZ: record.rotationRate.z)
            }
        }

        if self.motionManager.isAccelerometerAvailable {
            self.motionManager.startAccelerometerUpdates(to: q) {
                [weak self] (data, error) in
                guard let delegate = self?.delegate, let record = data, error == nil else {
                    return
                }
                delegate.motionDidAccelerometerUpdate(timestamp: record.timestamp, accelerationX: GRAVITY_NOMINAL*record.acceleration.x, accelerationY: GRAVITY_NOMINAL*record.acceleration.y, accelerationZ: GRAVITY_NOMINAL*record.acceleration.z)
            }
        }

        if useMagnetometer && self.motionManager.isMagnetometerAvailable {
            self.motionManager.startMagnetometerUpdates(to: q) {
                [weak self] (data, error) in
                guard let delegate = self?.delegate, let record = data, error == nil else {
                    return
                }
                delegate.motionDidMagnetometerUpdate(timestamp: record.timestamp, magnetFieldX: record.magneticField.x, magnetFieldY: record.magneticField.y, magnetFieldZ: record.magneticField.z)
            }
        }

        if useDeviceMotion && self.motionManager.isDeviceMotionAvailable {
            self.motionManager.startDeviceMotionUpdates(using: .xArbitraryZVertical, to: q) {
                [weak self] (data, error) in
                guard let delegate = self?.delegate, let record = data, error == nil else {
                    return
                }
                delegate.motionDidDeviceMotionUpdate(deviceMotion: record)
            }
        }

        if useAltimeter && CMAltimeter.isRelativeAltitudeAvailable() {
            self.altimeter.startRelativeAltitudeUpdates(to: q) {
                [weak self] (data, error) in
                guard let delegate = self?.delegate, let record = data, error == nil else {
                    return
                }
                delegate.motionDidAltimeterUpdate(timestamp: record.timestamp, pressure: record.pressure.doubleValue, relativeAltitude: record.relativeAltitude.doubleValue)
            }
        }
    }

    deinit {
        if CMAltimeter.isRelativeAltitudeAvailable() {
            self.altimeter.stopRelativeAltitudeUpdates()
        }
        if self.motionManager.isDeviceMotionActive {
            self.motionManager.stopDeviceMotionUpdates()
        }
        if self.motionManager.isMagnetometerActive {
            self.motionManager.stopMagnetometerUpdates()
        }
        if self.motionManager.isAccelerometerActive {
            self.motionManager.stopAccelerometerUpdates()
        }
        if self.motionManager.isGyroActive {
            self.motionManager.stopGyroUpdates()
        }
    }

    private let motionManager = CMMotionManager()
    private let altimeter = CMAltimeter()
}
