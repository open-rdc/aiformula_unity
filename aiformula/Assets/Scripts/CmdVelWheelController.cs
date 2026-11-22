using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;

[DisallowMultipleComponent]
public class CmdVelWheelController : MonoBehaviour
{
    [Header("ROS")]
    public string twistTopic = "/cmd_vel_twist";
    [Tooltip("Zero the command if no message arrives within this many seconds")]
    public float commandTimeout = 0.5f;

    [Header("Vehicle geometry (model.sdf DiffDrive plugin)")]
    public float wheelRadius = 0.124f;
    public float wheelSeparation = 0.6f;

    [Header("Wheel ArticulationBodies (auto-found by name if left empty)")]
    public ArticulationBody leftWheel;
    public ArticulationBody rightWheel;
    public string leftWheelName  = "left_wheel";
    public string rightWheelName = "right_wheel";

    [Header("Drive tuning")]
    [Tooltip("Velocity-tracking gain: torque = damping * (targetVel - vel)")]
    public float wheelDamping = 1.0e4f;
    [Tooltip("Max wheel torque [N*m]. URDF effort is 84.4; raise if it won't move")]
    public float wheelForceLimit = 1000f;
    [Tooltip("Flip to -1 if the robot drives backwards relative to the command")]
    public float driveDirection = 1f;
    [Tooltip("名前ではなく車体に対する実際の位置で左右の車輪を決める。" +
             "ai_car1 の URDF は left_wheel が右側 (y=-0.3)、right_wheel が左側 (y=+0.3) にあり名前が逆")]
    public bool detectSidesByPosition = true;

    [Header("Caster steering (Gazebo と同じく /cmd_caster の角度に位置制御)")]
    [Tooltip("true: steered_to_twist が出す /cmd_caster (data[0] = -steering_angle [rad]) の角度にキャスターを向ける。" +
             "ai_car1 のキャスターはトレールが無いので、自由回転にすると最初に切れた角度のまま戻らない")]
    public bool steerCaster = true;
    public string casterTopic = "/cmd_caster";
    [Tooltip("キャスターの最大切れ角 [deg]（URDF の limit 0.26 rad）")]
    public float casterMaxDeg = 15f;
    public float casterStiffness = 2000f;
    public float casterDamping = 100f;
    public float casterForceLimit = 1.0e4f;

    [Header("Caster (steerCaster=false のときだけ使う: 自由回転)")]
    public bool freeCaster = true;
    public string casterYawName  = "caster_yaw";
    public string casterRollName = "caster";
    public float casterYawDamping = 0.5f;

    [Header("Self-healing")]
    public bool forceMovableRoot = true;
    public bool disableUrdfImporterController = true;
    public bool autoCreateGround = true;
    public bool debugLog = true;

    ArticulationBody _root;
    float _targetLeftDegPerSec, _targetRightDegPerSec;
    float _lastMsgTime = -999f, _lastLogTime;
    Vector3 _lastLogPos;

    void Start()
    {
        _root = FindArticulationRoot();

        if (forceMovableRoot && _root != null)
        {
            _root.immovable = false;
            _root.useGravity = true;
        }
        if (disableUrdfImporterController) DisableImporterControllers();
        Physics.defaultSolverIterations = Mathf.Max(Physics.defaultSolverIterations, 30);

        if (leftWheel  == null) leftWheel  = FindBody(leftWheelName);
        if (rightWheel == null) rightWheel = FindBody(rightWheelName);
        if (leftWheel == null || rightWheel == null)
        {
            Debug.LogError($"[CmdVelWheelController] wheel ArticulationBody not found " +
                           $"(left='{leftWheelName}', right='{rightWheelName}'). " +
                           $"Assign them in the Inspector. Bodies present: {ListBodies()}");
            enabled = false;
            return;
        }

        if (detectSidesByPosition)
        {
            Transform rootT = _root != null ? _root.transform : transform;
            float lx = rootT.InverseTransformPoint(leftWheel.transform.position).x;
            float rx = rootT.InverseTransformPoint(rightWheel.transform.position).x;
            if (lx > rx)
            {
                (leftWheel, rightWheel) = (rightWheel, leftWheel);
                Debug.Log($"[CmdVelWheelController] 車輪の名前と実際の左右が逆なので入れ替えました: " +
                          $"left='{leftWheel.name}' (x={rx:F3}), right='{rightWheel.name}' (x={lx:F3})");
            }
        }

        ConfigureVelocityDrive(leftWheel);
        ConfigureVelocityDrive(rightWheel);
        _casterYaw = FindBody(casterYawName);
        if (steerCaster && _casterYaw != null)
        {
            ConfigureCasterSteer(_casterYaw);
            FreeSwivel(FindBody(casterRollName), 0f);
            ROSConnection.GetOrCreateInstance().Subscribe<Float64MultiArrayMsg>(casterTopic, OnCaster);
        }
        else if (freeCaster)
        {
            FreeSwivel(FindBody(casterYawName),  casterYawDamping);
            FreeSwivel(FindBody(casterRollName), 0f);
        }

        if (autoCreateGround) EnsureGround();

        _lastLogPos = _root != null ? _root.transform.position : transform.position;
        _lastLogTime = Time.time;

        ROSConnection.GetOrCreateInstance().Subscribe<TwistMsg>(twistTopic, OnTwist);
        Debug.Log($"[CmdVelWheelController] ready. root='{(_root ? _root.name : "?")}' " +
                  $"immovable={( _root && _root.immovable)} wheels='{leftWheel.name}','{rightWheel.name}' " +
                  $"topic='{twistTopic}'");
    }

    void OnTwist(TwistMsg msg)
    {
        float v = (float)msg.linear.x;
        float w = (float)msg.angular.z;
        float wL = (v - w * wheelSeparation * 0.5f) / wheelRadius;
        float wR = (v + w * wheelSeparation * 0.5f) / wheelRadius;
        _targetLeftDegPerSec  = wL * Mathf.Rad2Deg * driveDirection;
        _targetRightDegPerSec = wR * Mathf.Rad2Deg * driveDirection;
        _lastMsgTime = Time.time;
    }

    public void ResetCommand()
    {
        _targetLeftDegPerSec = _targetRightDegPerSec = 0f;
        _lastMsgTime = -999f;
    }

    void OnCaster(Float64MultiArrayMsg msg)
    {
        if (msg.data == null || msg.data.Length == 0) return;
        float steeringDeg = -(float)msg.data[0] * Mathf.Rad2Deg;
        _casterTargetDeg = Mathf.Clamp(steeringDeg, -casterMaxDeg, casterMaxDeg);
    }

    void FixedUpdate()
    {
        bool stale = (Time.time - _lastMsgTime) > commandTimeout;
        SetDriveVelocity(leftWheel,  stale ? 0f : _targetLeftDegPerSec);
        SetDriveVelocity(rightWheel, stale ? 0f : _targetRightDegPerSec);
        if (steerCaster && _casterYaw != null) UpdateCasterSteer();
    }

    ArticulationBody _casterYaw;
    Quaternion _casterRestRel;
    float _casterTargetDeg;
    float _casterSign = 1f;
    float _casterWrongSignTime;
    bool _casterSignChecked;

    void ConfigureCasterSteer(ArticulationBody body)
    {
        if (body.jointType != ArticulationJointType.RevoluteJoint)
            body.jointType = ArticulationJointType.RevoluteJoint;
        body.twistLock = ArticulationDofLock.LimitedMotion;
        var d = body.xDrive;
        d.driveType = ArticulationDriveType.Force;
        d.stiffness = casterStiffness;
        d.damping = casterDamping;
        d.forceLimit = casterForceLimit;
        d.lowerLimit = -casterMaxDeg - 1f;
        d.upperLimit = casterMaxDeg + 1f;
        d.target = 0f;
        d.targetVelocity = 0f;
        body.xDrive = d;

        Transform rootT = _root != null ? _root.transform : transform;
        _casterRestRel = Quaternion.Inverse(rootT.rotation) * body.transform.rotation;
        Vector3 axisWorld = body.transform.rotation * body.anchorRotation * Vector3.right;
        _casterSign = Vector3.Dot(axisWorld, rootT.up) >= 0f ? 1f : -1f;
        Debug.Log($"[CmdVelWheelController] caster '{body.name}' を位置制御にしました " +
                  $"(topic='{casterTopic}', ±{casterMaxDeg}deg, sign={_casterSign})");
    }

    float MeasuredCasterDeg()
    {
        Transform rootT = _root != null ? _root.transform : transform;
        Quaternion rel = Quaternion.Inverse(rootT.rotation) * _casterYaw.transform.rotation;
        Vector3 restFwdInCaster = Quaternion.Inverse(_casterRestRel) * Vector3.forward;
        Vector3 now = Vector3.ProjectOnPlane(rel * restFwdInCaster, Vector3.up);
        return Vector3.SignedAngle(Vector3.forward, now, Vector3.up);
    }

    void UpdateCasterSteer()
    {
        var d = _casterYaw.xDrive;
        d.target = _casterSign * _casterTargetDeg;
        _casterYaw.xDrive = d;

        if (_casterSignChecked || Mathf.Abs(_casterTargetDeg) < 3f) return;
        float measured = MeasuredCasterDeg();
        if (Mathf.Abs(measured) < 2f) return;
        if (Mathf.Sign(measured) == Mathf.Sign(_casterTargetDeg)) { _casterSignChecked = true; return; }
        _casterWrongSignTime += Time.fixedDeltaTime;
        if (_casterWrongSignTime > 0.3f)
        {
            _casterSign = -_casterSign;
            _casterSignChecked = true;
            Debug.LogWarning($"[CmdVelWheelController] キャスターの向きが指令と逆だったので符号を反転しました (sign={_casterSign})");
        }
    }

    void Update()
    {
        if (!debugLog || Time.time - _lastLogTime < 1f) return;
        _lastLogTime = Time.time;
        Vector3 pos = (_root ? _root.transform.position : transform.position);
        float lv = leftWheel.jointVelocity.dofCount > 0 ? leftWheel.jointVelocity[0] : 0f;
        float rv = rightWheel.jointVelocity.dofCount > 0 ? rightWheel.jointVelocity[0] : 0f;
        bool grounded = Physics.Raycast(pos + Vector3.up * 0.2f, Vector3.down, out var hit, 2f)
                        && hit.collider.transform.root != (_root ? _root.transform : transform);
        Debug.Log($"[CmdVelWheelController] tgtL/R={_targetLeftDegPerSec:F0}/{_targetRightDegPerSec:F0} deg/s | " +
                  $"wheelVel L/R={lv:F2}/{rv:F2} rad/s | grounded={grounded} | " +
                  (steerCaster && _casterYaw != null
                      ? $"caster tgt/now={_casterTargetDeg:F1}/{MeasuredCasterDeg():F1} deg | " : "") +
                  $"bodyMoved={(pos - _lastLogPos).magnitude:F3} m/s");
        _lastLogPos = pos;
    }

    void ConfigureVelocityDrive(ArticulationBody body)
    {
        if (body.jointType != ArticulationJointType.RevoluteJoint)
            body.jointType = ArticulationJointType.RevoluteJoint;
        body.twistLock = ArticulationDofLock.FreeMotion;

        var d = body.xDrive;
        d.driveType = ArticulationDriveType.Velocity;
        d.stiffness = 0f;
        d.damping = wheelDamping;
        d.forceLimit = wheelForceLimit;
        d.lowerLimit = 0f;
        d.upperLimit = 0f;
        d.target = 0f;
        d.targetVelocity = 0f;
        body.xDrive = d;
        body.jointFriction = 0f;
        body.angularDamping = 0f;

        Debug.Log($"[CmdVelWheelController] '{body.name}': jointType={body.jointType} " +
                  $"twistLock={body.twistLock} driveType={d.driveType} " +
                  $"damping={d.damping} forceLimit={d.forceLimit}");
    }

    void SetDriveVelocity(ArticulationBody body, float degPerSec)
    {
        var d = body.xDrive;
        d.targetVelocity = degPerSec;
        body.xDrive = d;
    }

    static void FreeSwivel(ArticulationBody body, float damping)
    {
        if (body == null) return;
        if (body.jointType == ArticulationJointType.FixedJoint)
            body.jointType = ArticulationJointType.RevoluteJoint;
        body.twistLock = ArticulationDofLock.FreeMotion;
        var d = body.xDrive;
        d.stiffness = 0f;
        d.damping = damping;
        d.forceLimit = 0f;
        d.lowerLimit = 0f;
        d.upperLimit = 0f;
        body.xDrive = d;
    }

    void DisableImporterControllers()
    {
        Transform root = _root != null ? _root.transform : transform;
        int n = 0;
        foreach (var mb in root.GetComponentsInChildren<MonoBehaviour>(true))
        {
            if (mb == null || mb == this) continue;
            string t = mb.GetType().FullName ?? "";
            if (t.StartsWith("Unity.Robotics.UrdfImporter.Control.") &&
                (t.EndsWith(".Controller") || t.EndsWith(".JointControl") || t.EndsWith(".FKRobot")))
            {
                mb.enabled = false;
                n++;
            }
        }
        if (n > 0) Debug.Log($"[CmdVelWheelController] disabled {n} URDF-Importer control component(s)");
    }

    void EnsureGround()
    {
        Bounds b = RobotBounds();
        Vector3 c = b.center;
        if (Physics.Raycast(new Vector3(c.x, b.max.y + 0.5f, c.z), Vector3.down, out var hit, 50f) &&
            hit.collider.transform.root != (_root ? _root.transform : transform))
            return;

        var g = GameObject.CreatePrimitive(PrimitiveType.Plane);
        g.name = "AutoGround (CmdVelWheelController)";
        g.transform.position = new Vector3(c.x, b.min.y - 0.002f, c.z);
        g.transform.localScale = new Vector3(30f, 1f, 30f);
        var pm = new PhysicMaterial("AutoHighFriction")
        {
            dynamicFriction = 1f,
            staticFriction = 1f,
            frictionCombine = PhysicMaterialCombine.Maximum
        };
        g.GetComponent<Collider>().material = pm;
        Debug.Log($"[CmdVelWheelController] no ground found -> created AutoGround at y={b.min.y - 0.002f:F3}");
    }

    Bounds RobotBounds()
    {
        Transform root = _root != null ? _root.transform : transform;
        var cols = root.GetComponentsInChildren<Collider>(true);
        if (cols.Length == 0) return new Bounds(root.position, Vector3.one);
        Bounds b = cols[0].bounds;
        for (int i = 1; i < cols.Length; i++) b.Encapsulate(cols[i].bounds);
        return b;
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

    ArticulationBody FindBody(string n)
    {
        Transform root = _root != null ? _root.transform : transform;
        foreach (var body in root.GetComponentsInChildren<ArticulationBody>(true))
            if (body.name == n) return body;
        return null;
    }

    string ListBodies()
    {
        Transform root = _root != null ? _root.transform : transform;
        var names = new List<string>();
        foreach (var body in root.GetComponentsInChildren<ArticulationBody>(true)) names.Add(body.name);
        return string.Join(", ", names);
    }
}
