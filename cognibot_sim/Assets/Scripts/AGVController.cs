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
    public float wheelRadius = 0.1f;     // meters
    public float trackWidth = 0.45f;      // distance between wheels
    public float maxLinearSpeed = 1f;      // m/s
    public float maxRotationalSpeed = 1f;  // rad/s
    public float forceLimit = 10f;
    public float damping = 10f;
    public float stiffness = 10000f;       // Added for better control

    [Header("ROS Settings")]
    public string topicName = "/cmd_vel";
    public float rosTimeout = 0.5f;  // seconds before zeroing

    private ArticulationBody leftAB;
    private ArticulationBody rightAB;
    private float linearInput = 0f;
    private float angularInput = 0f;
    private float lastCmdTime = -1f;  // Initialize to -1 to handle first message
    private bool isInitialized = false;

    private ROSConnection ros;

    void Start()
    {
        if (!ValidateComponents())
        {
            return;
        }

        InitializeWheels();
        InitializeROS();
        isInitialized = true;
    }

    bool ValidateComponents()
    {
        if (leftWheel == null || rightWheel == null)
        {
            Debug.LogError("Left and Right wheel GameObjects must be assigned.");
            return false;
        }

        // Get wheel articulation bodies
        leftAB = leftWheel.GetComponent<ArticulationBody>();
        rightAB = rightWheel.GetComponent<ArticulationBody>();

        if (leftAB == null || rightAB == null)
        {
            Debug.LogError("Wheel GameObjects must have ArticulationBody components.");
            return false;
        }

        return true;
    }

    void InitializeWheels()
    {
        InitDrive(leftAB);
        InitDrive(rightAB);
    }

    void InitializeROS()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TwistMsg>(topicName, OnCmdVelReceived);
    }

    void FixedUpdate()
    {
        if (!isInitialized) return;

        // Check for command timeout (skip timeout check for first message)
        if (lastCmdTime >= 0f && Time.fixedTime - lastCmdTime > rosTimeout)
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
        lastCmdTime = Time.fixedTime;  // Use fixedTime for consistency with FixedUpdate
        
        //Debug.Log($"Received command: linear={linearInput:F3}, angular={angularInput:F3}");
    }

    void InitDrive(ArticulationBody ab)
    {
        // Configure the appropriate drive based on joint type
        if (ab.jointType == ArticulationJointType.RevoluteJoint)
        {
            // For revolute joints, use the primary axis (typically the first one)
            var drive = ab.xDrive;
            drive.driveType = ArticulationDriveType.Velocity;
            drive.forceLimit = forceLimit;
            drive.damping = damping;
            drive.stiffness = stiffness;
            ab.xDrive = drive;
        }
        else
        {
            Debug.LogWarning($"Wheel {ab.name} is not a revolute joint. Check your URDF configuration.");
        }
    }

    void ApplyDriveCommands(float linear, float angular)
    {
        // Clamp inputs to limits
        linear = Mathf.Clamp(linear, -maxLinearSpeed, maxLinearSpeed);
        angular = Mathf.Clamp(angular, -maxRotationalSpeed, maxRotationalSpeed);

        // Differential drive kinematics
        // v_left = v_linear - w_angular * track_width/2
        // v_right = v_linear + w_angular * track_width/2
        float leftLinearSpeed = linear - angular * trackWidth / 2f;
        float rightLinearSpeed = linear + angular * trackWidth / 2f;

        // Convert to wheel angular velocities (rad/s)
        float leftWheelSpeed = leftLinearSpeed / wheelRadius;
        float rightWheelSpeed = rightLinearSpeed / wheelRadius;

        // Apply speed commands
        SetWheelSpeed(leftAB, leftWheelSpeed);
        SetWheelSpeed(rightAB, rightWheelSpeed);
         }

    void SetWheelSpeed(ArticulationBody ab, float radPerSec)
    {
        if (ab == null) return;

        // Unity ArticulationBody expects velocity in degrees per second
        var drive = ab.xDrive;
        drive.targetVelocity = radPerSec * Mathf.Rad2Deg;
        ab.xDrive = drive;
    }

    void OnDestroy()
    {
        // Clean up ROS subscription
        if (ros != null)
        {
            ros.Unsubscribe(topicName);
        }
    }

    
}