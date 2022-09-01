import UIKit

class App {
    class func loadSetting<T>(key: String, def: T) -> T {
        guard let object = UserDefaults.standard.object(forKey: key) else {
            return def
        }
        return object as! T
    }

    class func saveSetting<T>(key: String, value: T) {
        UserDefaults.standard.set(value, forKey: key)
    }

    class var keepAwake: Bool {
        get {
            return UIApplication.shared.isIdleTimerDisabled
        }
        set {
            UIApplication.shared.isIdleTimerDisabled = newValue
        }
    }

    class var documentDirectory: String {
        get {
            let paths = NSSearchPathForDirectoriesInDomains(.documentDirectory, .userDomainMask, true)
            return paths[0]
        }
    }

    class var currentDateTimeString: String {
        get {
            let datetime = Date()
            let formatter = DateFormatter()
            formatter.dateFormat = "yyyy-MM-dd-HH-mm-ss-SSS"
            return formatter.string(from: datetime)
        }
    }

}
