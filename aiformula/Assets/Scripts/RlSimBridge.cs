using System;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Nav;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;

[DisallowMultipleComponent]
public class RlSimBridge : MonoBehaviour
{
    [Header("ROS")]
    public string odomTopic = "/odom";
    public string resetTopic = "/sim/reset";
    public string odomFrameId = "odom";
    public string childFrameId = "ai_car1/chassis";
    [Tooltip("/odom の配信周期 [Hz]")]
    public float odomRate = 50f;

    [Header("Reset")]
    [Tooltip("空ならシーン開始時の位置・姿勢に戻す。指定するとその Transform の位置・姿勢に戻す")]
    public Transform resetPoint;
    [Tooltip("リセット時に少し浮かせて地面へのめり込みを防ぐ [m]")]
    public float resetLift = 0.02f;
    [Tooltip("Game ビューでこのキーを押すと車をスタート位置に戻す（手動リセット用）。None で無効")]
    public KeyCode resetKey = KeyCode.R;

    ROSConnection _ros;
    ArticulationBody _root;
    ArticulationBody[] _bodies;
    CmdVelWheelController _wheel;
    Vector3 _startPos;
    Quaternion _startRot;
    Vector3 _odomOrigin;
    Quaternion _odomOriginRot;
    float _nextOdom;
    bool _resetRequested;

    void Start()
    {
        _root = FindArticulationRoot();
        if (_root == null)
        {
            Debug.LogError("[RlSimBridge] ArticulationBody が見つかりません。ai_car1 ルートに付けてください");
            enabled = false;
            return;
        }
        _bodies = _root.GetComponentsInChildren<ArticulationBody>(true);
        _wheel = GetComponentInParent<CmdVelWheelController>() ?? _root.GetComponentInChildren<CmdVelWheelController>(true);

        _startPos = _root.transform.position;
        _startRot = _root.transform.rotation;
        _odomOrigin = _startPos;
        _odomOriginRot = _startRot;

        _ros = ROSConnection.GetOrCreateInstance();
        _ros.RegisterPublisher<OdometryMsg>(odomTopic);
        _ros.Subscribe<EmptyMsg>(resetTopic, _ => _resetRequested = true);
        Debug.Log($"[RlSimBridge] ready. root='{_root.name}' odom='{odomTopic}' reset='{resetTopic}'");
    }

    void Update()
    {
        if (resetKey != KeyCode.None && Input.GetKeyDown(resetKey)) _resetRequested = true;
    }

    void FixedUpdate()
    {
        if (_resetRequested)
        {
            _resetRequested = false;
            DoReset();
        }

        if (Time.time < _nextOdom) return;
        _nextOdom = Time.time + 1f / Mathf.Max(odomRate, 1f);
        PublishOdom();
    }

    void DoReset()
    {
        Vector3 pos = resetPoint ? resetPoint.position : _startPos;
        Quaternion rot = resetPoint ? resetPoint.rotation : _startRot;
        if (_wheel != null) _wheel.ResetCommand();

        _root.TeleportRoot(pos + Vector3.up * resetLift, rot);
        foreach (var b in _bodies)
        {
            b.velocity = Vector3.zero;
            b.angularVelocity = Vector3.zero;
            if (!b.isRoot && b.dofCount > 0)
            {
                var jv = b.jointVelocity;
                for (int i = 0; i < jv.dofCount; i++) jv[i] = 0f;
                b.jointVelocity = jv;
            }
        }
        Debug.Log($"[RlSimBridge] reset -> pos={pos} yaw={rot.eulerAngles.y:F1}");
    }

    void PublishOdom()
    {
        Transform t = _root.transform;
        Vector3 relPos = Quaternion.Inverse(_odomOriginRot) * (t.position - _odomOrigin);
        Quaternion relRot = Quaternion.Inverse(_odomOriginRot) * t.rotation;
        Vector3 vLocal = t.InverseTransformDirection(_root.velocity);
        Vector3 wLocal = t.InverseTransformDirection(_root.angularVelocity);

        double now = Time.timeAsDouble;
        int sec = (int)Math.Floor(now);
        var stamp = new TimeMsg(sec, (uint)((now - sec) * 1e9));

        var msg = new OdometryMsg
        {
            header = new HeaderMsg { stamp = stamp, frame_id = odomFrameId },
            child_frame_id = childFrameId,
        };
        msg.pose.pose.position = relPos.To<FLU>();
        msg.pose.pose.orientation = relRot.To<FLU>();
        msg.twist.twist.linear = vLocal.To<FLU>();
        Vector3<FLU> w = wLocal.To<FLU>();
        msg.twist.twist.angular = new RosMessageTypes.Geometry.Vector3Msg(-w.x, -w.y, -w.z);
        _ros.Publish(odomTopic, msg);
    }

    ArticulationBody FindArticulationRoot()
    {
        var b = GetComponentInParent<ArticulationBody>();
        if (b == null) b = GetComponentInChildren<ArticulationBody>(true);
        if (b == null) return null;
        while (!b.isRoot && b.transform.parent != null)
        {
            var p = b.transform.parent.GetComponentInParent<ArticulationBody>();
            if (p == null) break;
            b = p;
        }
        return b;
    }
}
