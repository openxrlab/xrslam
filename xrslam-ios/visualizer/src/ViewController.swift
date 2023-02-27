import UIKit
import SceneKit
import AVFoundation
import AudioToolbox

class ViewController: UIViewController, SCNSceneRendererDelegate {

    var xrslamer: XRSLAMer? = nil
    var camera: Camera? = nil
    var motion: Motion? = nil

    var scene: SCNScene!
    var sceneCameraNode: SCNNode!

    var virtualObjectNodes: [SCNNode] = []

    let colors: [UIColor] = [UIColor.blue, UIColor.brown, UIColor.green, UIColor.white, UIColor.cyan, UIColor.gray, UIColor.orange, UIColor.darkGray, UIColor.magenta, UIColor.red]

    @IBOutlet var previewView: UIView!
    @IBOutlet var sceneView: SCNView!

    @IBOutlet weak var button_start: UIButton!
    @IBOutlet weak var button_reset: UIButton!
    @IBOutlet weak var button_debug: UIButton!
    @IBOutlet weak var button_switch: UISegmentedControl!
    @IBOutlet weak var textUI: UIStackView!
    @IBOutlet weak var loggerText: UILabel!

//    @IBOutlet weak var label_state: UILabel!
    @IBOutlet weak var message: UITextView!

    var system_mode = 0
    var system_state = -2
    var enable_loc = false
    var message_list: Array<String> = Array()
    var type_list: Array<Character> = Array()

    func material(withColor color : UIColor) -> SCNMaterial {
        let material = SCNMaterial();
        material.diffuse.contents = color
        material.locksAmbientWithDiffuse = true
        return material
    }

    @IBAction func tappedScreen(_ sender: UITapGestureRecognizer) {
        guard sender.view != nil else { return }

        if (sender.state == .ended) {
            let created: Bool = self.xrslamer?.createVirtualObject() ?? false
            print("createVirtualObject: \(created)")
            if created {
//                if let scn = SCNScene.init(named: "scn/mac.scn"), let node = scn.rootNode.childNode(withName: "laptop", recursively: true)?.clone() {
//                if let scn = SCNScene.init(named: "scn/XRLAB.scn"), let node = scn.rootNode.childNode(withName: "XRLAB", recursively: true)?.clone() {
//                    if let xrslamer = self.xrslamer {
//                        let size = xrslamer.virtualObjectPositions.count
//                        node.position = xrslamer.virtualObjectPositions[size-1]
//                        node.orientation = xrslamer.virtualObjectRotations[size-1]
//                    }
//                    virtualObjectNodes.append(node)
//                    self.sceneView.scene?.rootNode.addChildNode(node)
//                }
                let length: Float = 0.5
                let cube = SCNBox(width: CGFloat(length), height: CGFloat(length), length: CGFloat(length), chamferRadius: 0)
                let greenMaterial = material(withColor: UIColor.green)
                let redMaterial = material(withColor: UIColor.red)
                let blueMaterial = material(withColor: UIColor.blue)
                // front right back left top bottom
                let type: Int = Int(arc4random()) % 3
                if (type == 0) {
                    cube.materials = [greenMaterial, redMaterial, greenMaterial, redMaterial, blueMaterial, blueMaterial];
                }
                else if (type == 1) {
                    cube.materials = [redMaterial, greenMaterial, redMaterial, greenMaterial, blueMaterial, blueMaterial];
                }
                else{
                    cube.materials = [blueMaterial, greenMaterial, blueMaterial, greenMaterial, redMaterial, redMaterial];
                }

                let cubeNode = SCNNode(geometry: cube)
                cubeNode.pivot = SCNMatrix4MakeTranslation(0.0, -0.5 * length, 0.0)
                if let xrslamer = self.xrslamer {
                    let size = xrslamer.virtualObjectPositions.count
                    cubeNode.position = xrslamer.virtualObjectPositions[size-1]
                    cubeNode.orientation = xrslamer.virtualObjectRotations[size-1]
                }
                virtualObjectNodes.append(cubeNode)
                self.sceneView.scene?.rootNode.addChildNode(cubeNode)
            }
        }
    }

    override func viewDidLoad() {
        super.viewDidLoad()
        App.keepAwake = true

        setupScene()
        message.isEditable = false

        self.sceneView.delegate = self
        self.sceneView.isPlaying = true
        self.sceneView.loops = true

        NotificationCenter.default.addObserver(self, selector: #selector(applicationDidBecomeActive), name: UIApplication.didBecomeActiveNotification, object: nil)
        NotificationCenter.default.addObserver(self, selector: #selector(applicationWillResignActive), name: UIApplication.willResignActiveNotification, object: nil)

        textUI.clipsToBounds = true
        textUI.layer.cornerRadius = 10
        textUI.layer.maskedCorners = [.layerMaxXMinYCorner, .layerMinXMinYCorner]
        loggerText.textAlignment = .center

        button_start.backgroundColor = UIColor.systemRed;
        button_debug.backgroundColor = UIColor.systemRed;
        button_reset.backgroundColor = UIColor.systemRed;
        button_start.layer.cornerRadius = 10;
        button_debug.layer.cornerRadius = 10;
        button_reset.layer.cornerRadius = 10;
    }

    override var shouldAutorotate: Bool {
        return false
    }

    func myPrint(type: Character, context: String) {
        message_list.append(context)
        type_list.append(type)

        DispatchQueue.main.async {
            self.updateText()
        }
    }

    func updateText() {
        message.text = ""
        let len = message_list.count
        for (idx, m) in message_list.enumerated() {
            if len - idx > 10{
                continue
            }
            message.text += String(idx + 1) + " | " + m + "\n"
        }
        let range = NSRange(location: message.text.count - 1, length: 0)
        message.scrollRangeToVisible(range)
    }

    func showToast(message: String) {
        var style = ToastStyle()
        style.messageAlignment = .center
        style.horizontalPadding = 15.0
        style.verticalPadding = 15.0
        style.backgroundColor = UIColor.white.withAlphaComponent(0.08)
        style.messageColor = .black
        style.messageFont = .monospacedDigitSystemFont(ofSize: 18.0, weight: UIFont.Weight.bold)
        style.fadeDuration = 0
        self.view.makeToast(message, duration: 0.05, position: .center, style: style)
    }

    @objc func applicationDidBecomeActive() {
        startCamera()
    }

    @objc func applicationWillResignActive() {
        stopCamera()
    }

    func renderer(_ renderer: SCNSceneRenderer, updateAtTime time: TimeInterval) {
        if let state = self.xrslamer?.getSystemState(), let isrun = self.xrslamer?.isRuning() {
            let nstate = isrun ? state: -1
            if nstate != system_state {
                var nmessage: String
                switch nstate {
                case -1: nmessage = "SLAM:Start"
                case 0: nmessage = "SLAM:Initializing"
                case 1: nmessage = "SLAM:Tracking"
                case 2: nmessage = "SLAM:Crash"
                default: nmessage = ""
                }

                myPrint(type: "I", context: nmessage)
            }
            if nstate == 0 {
                DispatchQueue.main.async {
                    var msg_list: Array<String> = Array()
                    msg_list.append("\nStep 1: Initializing SLAM\n")
                    var msg: String = ""
                    for mm in msg_list {
                        msg += mm
                    }
                    self.showToast(message: msg)
                }
            }
            if nstate == 1 {
                if system_mode == 1{
                    if let n_vloc_state = xrslamer?.global_localization_initialized() {
                        if n_vloc_state == 0 {
                            DispatchQueue.main.async {
                                var msg_list: Array<String> = Array()
                                msg_list.append("Step 2: Trying to localize your position\n\n")
                                msg_list.append("Please keep your phone facing the front")
                                var msg: String = ""
                                for mm in msg_list {
                                    msg += mm
                                }
                                self.showToast(message: msg)
                            }
                        }
                    }
                }
            }
            system_state = nstate

            if let strs = xrslamer?.get_logger_message() {
                for (_, s) in strs.enumerated() {
                    let str:String = s as! String
                    myPrint(type: "I", context: str)
                }
            }

        }

        if let xrslamer = self.xrslamer {
            self.sceneView.pointOfView?.position = xrslamer.getCameraPosition()
            self.sceneView.pointOfView?.orientation = xrslamer.getCameraRotation()
            self.scene.background.contents = xrslamer.getUIImage()
        }

    }

    func setupScene() {
        scene = SCNScene()
        scene.background.contentsTransform = SCNMatrix4MakeRotation(Float.pi / 2, 0.0, 0.0, 1.0)

        self.sceneView.scene = scene
        self.sceneView.autoenablesDefaultLighting = true
        self.sceneView.allowsCameraControl = false
        self.sceneView.preferredFramesPerSecond = 30

        sceneCameraNode = SCNNode()
        sceneCameraNode.camera = SCNCamera()
        sceneCameraNode.camera?.zNear = 0.0001
        sceneCameraNode.camera?.zFar = 20
        sceneCameraNode.position = SCNVector3(0, 0, 0)
        sceneCameraNode.orientation = SCNQuaternion(0, 0, 0, 1)

        self.sceneView.pointOfView = sceneCameraNode
        self.sceneView.scene?.rootNode.addChildNode(sceneCameraNode)
        self.sceneView.backgroundColor = UIColor.clear
    }

    func startCamera() {
        guard let camera = Camera() else {
            NSLog("Cannot access camera")
            return
        }

        camera.setFps(30)
        camera.setFocus(0.835)

        guard let motion = Motion() else {
           NSLog("Cannot access motion")
            return
        }

        self.xrslamer = XRSLAMer()
        self.camera = camera
        self.motion = motion
        self.motion?.delegate = xrslamer
        self.camera?.delegate = xrslamer

        if system_mode == 0{
            self.xrslamer?.DisableLoc()
        }else{
            self.xrslamer?.EnableLoc()
        }

        let imageView: UIImageView = UIImageView()
        imageView.frame = self.previewView.bounds
        self.previewView.addSubview(imageView)
        self.xrslamer?.targetView = imageView

        self.sceneCameraNode.camera?.fieldOfView = CGFloat(self.xrslamer?.fov ?? 49)
        print("xrslamer fov \(String(describing: self.xrslamer?.fov))")
    }

    func stopCamera() {
        self.motion = nil
        self.camera = nil
        self.xrslamer = nil
        self.sceneView.scene?.background.contents = nil
    }

    @IBAction func clickStartButton(_ sender: UIButton) {
        AudioServicesPlaySystemSound(1519)
        if let state = self.xrslamer?.isRuning(), state == true {
            print("you tapped the START button, but the system is running")
            myPrint(type: "I", context: "Already Start!")
        }else {
            print("Start!");
            self.xrslamer = nil
            startCamera()
            self.xrslamer?.Begin()
            button_switch.setEnabled(false, forSegmentAt: 0)
            button_switch.setEnabled(false, forSegmentAt: 1)
        }
    }

    @IBAction func clickResetButton(_ sender: UIButton) {
        AudioServicesPlaySystemSound(1519)
        if let state = self.xrslamer?.isRuning(), state == false {
            print("you tapped the RESET button, but the system is already reset!")
            myPrint(type: "I", context: "Already Reset!")
        }else {
            print("Reset!");
            message_list.removeAll()
            myPrint(type: "I", context: "Reset!")
            system_state = -1
            self.xrslamer = nil
            startCamera()
            for (i, _) in virtualObjectNodes.enumerated() {
                virtualObjectNodes[i].removeFromParentNode()
            }
            virtualObjectNodes = []
            self.xrslamer?.Stop()

            button_switch.setEnabled(true, forSegmentAt: 0)
            button_switch.setEnabled(true, forSegmentAt: 1)
        }
    }

    @IBAction func clickSegmentationControl(_ sender: UISegmentedControl) {
        AudioServicesPlaySystemSound(1519)
        if let state = self.xrslamer?.isRuning(), state == true{
            print("you want to switch the System between SLAM/VLoc., but the System is running")
            myPrint(type: "I", context: "Cannot Switch Mode During Running!")
        }else{
            switch button_switch.selectedSegmentIndex{
            case 0:
                print("Switch to SLAM Mode")
                myPrint(type: "I", context: "SLAM Mode")
                system_mode = 0
            case 1:
                print("Switch to VLoc. Mode")
                myPrint(type: "I", context: "VLoc. Mode")
                system_mode = 1
            default:
                break
            }
        }
    }

    @IBAction func clickDebugButton(_ sender: UIButton) {
        AudioServicesPlaySystemSound(1519)
        if(system_mode == 1) {
            myPrint(type: "I", context: "Query Frame")
            xrslamer?.query_frame()
        }
        else{
            myPrint(type: "I", context: "SLAM Mode Do not support visual localization!")
        }
    }

}
