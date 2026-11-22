using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Nav;

[DisallowMultipleComponent]
public class OdomSubscriber : MonoBehaviour
{
    [Tooltip("Odometry topic to follow")]
    public string topic = "/odom";

    public bool applyPosition = true;
    public bool applyRotation = true;

    ArticulationBody _rootBody;
    Vector3 _sceneOrigin;
    bool _haveOrigin;

    void Start()
    {
        _rootBody = GetComponent<ArticulationBody>();
        if (_rootBody != null && !_rootBody.isRoot) _rootBody = null;
        _sceneOrigin = transform.position;

        ROSConnection.GetOrCreateInstance().Subscribe<OdometryMsg>(topic, OnOdom);
    }

    void OnOdom(OdometryMsg msg)
    {
        Vector3 p = msg.pose.pose.position.From<FLU>();
        Quaternion q = msg.pose.pose.orientation.From<FLU>();

        if (!_haveOrigin) { _sceneOrigin -= p; _haveOrigin = true; }

        Vector3 targetPos = applyPosition ? _sceneOrigin + p : transform.position;
        Quaternion targetRot = applyRotation ? q : transform.rotation;

        if (_rootBody != null)
            _rootBody.TeleportRoot(targetPos, targetRot);
        else
        {
            transform.position = targetPos;
            transform.rotation = targetRot;
        }
    }
}
