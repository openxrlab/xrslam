import VideoToolbox

class VTVideoCodecType {
    static let H264 = VTVideoCodecType(kCMVideoCodecType_H264)
    static let HEVC = VTVideoCodecType(kCMVideoCodecType_HEVC)
    let kCMVideoCodecType: CMVideoCodecType
    init(_ value: CMVideoCodecType) {
        self.kCMVideoCodecType = value
    }
}

extension CMVideoFormatDescription {
    func getH264ParameterSet(at index: Int) -> Data? {
        var pointer: UnsafePointer<UInt8>? = nil
        var size = Int()

        guard CMVideoFormatDescriptionGetH264ParameterSetAtIndex(self, parameterSetIndex: index, parameterSetPointerOut: &pointer, parameterSetSizeOut: &size, parameterSetCountOut: nil, nalUnitHeaderLengthOut: nil) == noErr else {
            return nil
        }

        return Data(bytes: pointer!, count: size)
    }
}

extension CMSampleBuffer {
    var dataIsReady: Bool {
        get {
            return CMSampleBufferDataIsReady(self)
        }
    }

    var presentationTimeStamp: CMTime {
        get {
            return CMSampleBufferGetPresentationTimeStamp(self)
        }
    }

    var formatDescription: CMFormatDescription? {
        get {
            return CMSampleBufferGetFormatDescription(self)
        }
    }

    var imageBuffer: CVImageBuffer? {
        get {
            return CMSampleBufferGetImageBuffer(self)
        }
    }

    var dataBuffer: CMBlockBuffer? {
        get {
            return CMSampleBufferGetDataBuffer(self)
        }
    }

    func getSampleAttachmentsArray(createIfNecessary: Bool) -> Array<AnyObject>? {
        return CMSampleBufferGetSampleAttachmentsArray(
            self,
            createIfNecessary: createIfNecessary) as Array<AnyObject>?
    }

    var sampleAttachmentsArray: Array<AnyObject>! {
        get {
            return CMSampleBufferGetSampleAttachmentsArray(self, createIfNecessary: true)! as Array<AnyObject>
        }
    }
}

extension CVPixelBuffer {
    var width: Int {
        get {
            return CVPixelBufferGetWidth(self)
        }
    }

    var height: Int {
        get {
            return CVPixelBufferGetHeight(self)
        }
    }

    func lockBaseAddress(_ lockFlags: CVPixelBufferLockFlags) {
        CVPixelBufferLockBaseAddress(self, lockFlags)
    }

    func unlockBaseAddress(_ unlockFlags: CVPixelBufferLockFlags) {
        CVPixelBufferUnlockBaseAddress(self, unlockFlags)
    }

    func getBaseAddress() -> UnsafeMutableRawPointer? {
        return CVPixelBufferGetBaseAddress(self)
    }

    func getBaseAddress(ofPlane: Int) -> UnsafeMutableRawPointer? {
        return CVPixelBufferGetBaseAddressOfPlane(self, ofPlane)
    }

    func getBytesPerRow() -> Int {
        return CVPixelBufferGetBytesPerRow(self)
    }

    func getBytesPerRow(ofPlane: Int) -> Int {
        return CVPixelBufferGetBytesPerRowOfPlane(self, ofPlane)
    }
}

extension CMBlockBuffer {
    @discardableResult func getDataPointer(
        _ atOffset: Int,
        lengthAtOffsetOut: UnsafeMutablePointer<Int>?,
        totalLengthOut: UnsafeMutablePointer<Int>?,
        dataPointerOut: UnsafeMutablePointer<UnsafeMutablePointer<Int8>?>?) -> OSStatus {
        return CMBlockBufferGetDataPointer(
            self,
            atOffset: atOffset,
            lengthAtOffsetOut: lengthAtOffsetOut,
            totalLengthOut: totalLengthOut,
            dataPointerOut: dataPointerOut)
    }
}

extension VTCompressionSession {
    class func create(width: Int, height: Int, codecType: VTVideoCodecType) -> VTCompressionSession? {
        return create(width: width, height: height, codecType: codecType, outputCallback: nil)
    }

    class func create(width: Int, height: Int, codecType: VTVideoCodecType, outputCallback: VTCompressionOutputCallback?) -> VTCompressionSession? {
        return create(width: width, height: height, codecType: codecType, outputCallbackRefCon: nil, outputCallback: outputCallback)
    }

    class func create(width: Int, height: Int, codecType: VTVideoCodecType, outputCallbackRefCon: UnsafeMutableRawPointer?, outputCallback: VTCompressionOutputCallback?) -> VTCompressionSession? {
        return create(width: width, height: height, codecType: codecType, encoderSpecification: nil, imageBufferAttributes: nil, outputCallbackRefCon: outputCallbackRefCon, outputCallback: outputCallback)
    }

    class func create(width: Int, height: Int, codecType: VTVideoCodecType, encoderSpecification: Dictionary<String, AnyObject>?, imageBufferAttributes: Dictionary<String, AnyObject>?) -> VTCompressionSession? {
        return create(width: width, height: height, codecType: codecType, encoderSpecification: encoderSpecification, imageBufferAttributes: imageBufferAttributes, outputCallback: nil)
    }

    class func create(width: Int, height: Int, codecType: VTVideoCodecType, encoderSpecification: Dictionary<String, AnyObject>?, imageBufferAttributes: Dictionary<String, AnyObject>?, outputCallback: VTCompressionOutputCallback?) -> VTCompressionSession? {
        return create(width: width, height: height, codecType: codecType, encoderSpecification: encoderSpecification, imageBufferAttributes: imageBufferAttributes, outputCallbackRefCon: nil, outputCallback: outputCallback)
    }

    class func create(width: Int, height: Int, codecType: VTVideoCodecType, encoderSpecification: Dictionary<String, AnyObject>?, imageBufferAttributes: Dictionary<String, AnyObject>?, outputCallbackRefCon: UnsafeMutableRawPointer?, outputCallback: VTCompressionOutputCallback?) -> VTCompressionSession? {
        var session: VTCompressionSession? = nil
        guard VTCompressionSessionCreate(
            allocator: nil,
            width: Int32(width),
            height: Int32(height),
            codecType: codecType.kCMVideoCodecType,
            encoderSpecification: encoderSpecification as CFDictionary?,
            imageBufferAttributes: imageBufferAttributes as CFDictionary?,
            compressedDataAllocator: nil,
            outputCallback: outputCallback,
            refcon: outputCallbackRefCon,
            compressionSessionOut: &session) == noErr else {
                return nil
        }
        return session
    }

    @discardableResult func setProperties(_ dictionary: Dictionary<String, AnyObject>) -> OSStatus {
        return VTSessionSetProperties(self, propertyDictionary: dictionary as CFDictionary)
    }

    @discardableResult func prepareToEncodeFrames() -> OSStatus {
        return VTCompressionSessionPrepareToEncodeFrames(self)
    }

    @discardableResult func encodeFrame(
        _ imageBuffer: CVImageBuffer,
        presentationTimeStamp: CMTime,
        duration: CMTime,
        frameProperties: Dictionary<String, AnyObject>?,
        infoFlagsOut: UnsafeMutablePointer<VTEncodeInfoFlags>?,
        outputHandler: @escaping VTCompressionOutputHandler) -> OSStatus {
        return VTCompressionSessionEncodeFrame(
            self,
            imageBuffer: imageBuffer,
            presentationTimeStamp: presentationTimeStamp,
            duration: duration,
            frameProperties: frameProperties as CFDictionary?,
            infoFlagsOut: infoFlagsOut,
            outputHandler: outputHandler)
    }

    @discardableResult func encodeFrame(
        _ imageBuffer: CVImageBuffer,
        presentationTimeStamp: CMTime,
        duration: CMTime,
        frameProperties: Dictionary<String, AnyObject>?,
        sourceFrameRefcon: UnsafeMutableRawPointer?,
        infoFlagsOut: UnsafeMutablePointer<VTEncodeInfoFlags>?
        ) -> OSStatus {
        return VTCompressionSessionEncodeFrame(
            self,
            imageBuffer: imageBuffer,
            presentationTimeStamp: presentationTimeStamp,
            duration: duration,
            frameProperties: frameProperties as CFDictionary?,
            sourceFrameRefcon: sourceFrameRefcon,
            infoFlagsOut: infoFlagsOut)
    }

    @discardableResult func completeFrames(until presentationTimeStamp: CMTime) -> OSStatus {
        return VTCompressionSessionCompleteFrames(
            self,
            untilPresentationTimeStamp: presentationTimeStamp)
    }

    func invalidate() {
        VTCompressionSessionInvalidate(self)
    }
}
