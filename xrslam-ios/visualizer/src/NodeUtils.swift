import UIKit
import SceneKit

func createCubeNode(_ width: Double, _ height: Double, _ length: Double, _ chamferRadius: Double = 0.0) -> SCNNode {
    let boxGeometry = SCNBox(width: CGFloat(width), height: CGFloat(height), length: CGFloat(length), chamferRadius: CGFloat(chamferRadius))

    let node = SCNNode(geometry: boxGeometry)
    node.name = "cube"
    return node
}

func createAxesNode(_ scale: Double = 1.0) -> SCNNode {
    let xGeometry = SCNCapsule(capRadius: 0.05, height: CGFloat(scale * 10.0))
    xGeometry.firstMaterial?.diffuse.contents = UIColor.red
    let yGeometry = SCNCapsule(capRadius: 0.05, height: CGFloat(scale * 10.0))
    yGeometry.firstMaterial?.diffuse.contents = UIColor.green
    let zGeometry = SCNCapsule(capRadius: 0.05, height: CGFloat(scale * 10.0))
    zGeometry.firstMaterial?.diffuse.contents = UIColor.blue

    let xAxis = SCNNode(geometry: xGeometry)
    xAxis.name = "xAxis"
    xAxis.position = SCNVector3(scale * 5.0, 0.0, 0.0)
    xAxis.orientation = SCNQuaternion(0, 0, 0.7071068, 0.7071068)
    let yAxis = SCNNode(geometry: yGeometry)
    yAxis.name = "yAxis"
    yAxis.orientation = SCNQuaternion(0.0, 0.0, 0.0, 1.0)
    yAxis.position = SCNVector3(0.0, scale * 5.0, 0.0)
    let zAxis = SCNNode(geometry: zGeometry)
    zAxis.name = "zAxis"
    zAxis.orientation = SCNQuaternion(0.7071068, 0, 0, 0.7071068)
    zAxis.position = SCNVector3(0.0, 0.0, scale * 5.0)

    let axes = SCNNode()
    axes.name = "axes"
    axes.addChildNode(xAxis)
    axes.addChildNode(yAxis)
    axes.addChildNode(zAxis)

    return axes
}
