#import <Foundation/Foundation.h>
#import <SceneKit/SceneKit.h>
#import <UIKit/UIKit.h>
#import <AVFoundation/AVFoundation.h>

NS_ASSUME_NONNULL_BEGIN

@interface XRSLAM : NSObject

typedef NS_ENUM(NSUInteger, SysState) {
    SYS_INITIALIZING = 0,
    SYS_TRACKING,
    SYS_CRASH,
    SYS_UNKNOWN
};
- (id)init:(NSString *)model;
- (void)dealloc;
- (void)processBuffer:(CMSampleBufferRef)buffer;
- (void)trackGyroscope:(double)t x:(double)x y:(double)y z:(double)z;
- (void)trackAccelerometer:(double)t x:(double)x y:(double)y z:(double)z;
- (void)trackCamera:(double)t buffer:(CMSampleBufferRef)buffer;
- (double)getFOV;
- (SCNVector3)getCameraPosition;
- (SCNQuaternion)getCameraRotation;
- (size_t)createVirtualObject;
- (void)updateVirtualObject:(size_t)object_id;
- (SCNVector3)getVirtualObjectPosition;
- (SCNQuaternion)getVirtualObjectRotation;
- (int)getVirtualObjectIsolated;
- (SysState)get_system_state;
- (void)enable_global_localization;
- (void)disable_global_localization;
- (int)global_localization_initialized;
- (void)query_frame;
- (NSMutableArray *)get_logger_message;
- (UIImage *)getLatestImage;
- (UIImage *)getCurrentImage;
- (void)updateImage:(double)t image:(UIImage *)image;

@end

NS_ASSUME_NONNULL_END
