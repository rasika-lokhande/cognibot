using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using Unity.Robotics.UrdfImporter;

public class AGVController : MonoBehaviour
{
    [Header("Wheel GameObjects")]
    public GameObject leftWheel;
    public GameObject rightWheel;

    [Header("Control Parameters")]
    public float wheelRadius = 0.033f;     // meters
    public float trackWidth = 0.288f;      // distance between wheels
    public float maxLinearSpeed = 2f;      // m/s
    public float maxRotationalSpeed = 1f;  // rad/s
    public float forceLimit = 10f;
    public float damping = 10f;

    [Header("ROS Settings")]
    public string topicName = "/cmd_vel";
    public float rosTimeout = 0.5f;  // seconds before zeroing

    private ArticulationBody leftAB;
    private ArticulationBody rightAB;
    private float linearInput = 0f;
    private float angularInput = 0f;
    private float lastCmdTime = 0f;

    private ROSConnection ros;

    void Start()
    {
        // Get wheel articulation bodies
        leftAB = leftWheel.GetComponent<ArticulationBody>();
        rightAB = rightWheel.GetComponent<ArticulationBody>();

        InitDrive(leftAB);
        InitDrive(rightAB);

        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TwistMsg>(topicName, OnCmdVelReceived);
    }

    void FixedUpdate()
    {
        if (Time.time - lastCmdTime > rosTimeout)
        {
            linearInput = 0f;
            angularInput = 0f;
        }

        ApplyDriveCommands(linearInput, angularInput);
      
    }

    void OnCmdVelReceived(TwistMsg msg)
    {
        linearInput = (float)msg.linear.x;
        angularInput = (float)msg.angular.z;
        lastCmdTime = Time.time;
    }

    void InitDrive(ArticulationBody ab)
    {
        var drive = ab.xDrive;
        drive.forceLimit = forceLimit;
        drive.damping = damping;
        ab.xDrive = drive;
    }

    void ApplyDriveCommands(float linear, float angular)
    {
        // Clamp to limits
        linear = Mathf.Clamp(linear, -maxLinearSpeed, maxLinearSpeed);
        angular = Mathf.Clamp(angular, -maxRotationalSpeed, maxRotationalSpeed);

        // Calculate wheel speeds in rad/s
        float leftSpeed = (linear - angular * trackWidth / 2f) / wheelRadius;
        float rightSpeed = (linear + angular * trackWidth / 2f) / wheelRadius;

        SetWheelSpeed(leftAB, leftSpeed);
        SetWheelSpeed(rightAB, rightSpeed);
    }

    void SetWheelSpeed(ArticulationBody ab, float radPerSec)
    {
        var drive = ab.xDrive;
        drive.targetVelocity = radPerSec * Mathf.Rad2Deg; // Unity expects deg/sec
        ab.xDrive = drive;
    }
}
