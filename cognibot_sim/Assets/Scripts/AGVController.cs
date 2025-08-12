using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class AGVController : MonoBehaviour
{
    [Header("Configuration")]
    public GameObject leftWheel;
    public GameObject rightWheel;
    public float wheelRadius = 0.1f;
    public float trackWidth = 0.45f;
    public float maxSpeed = 2f;
    public float forceLimit = 10f;
    public float damping = 10f;
    public string topicName = "/cmd_vel";
    public float timeout = 0.5f;

    private ArticulationBody leftAB, rightAB;
    private Vector2 command; // x=linear, y=angular
    private float lastCmdTime = -1f;

    void Start()
    {
        // Get components with validation
        leftAB = leftWheel?.GetComponent<ArticulationBody>();
        rightAB = rightWheel?.GetComponent<ArticulationBody>();
        
        if (!leftAB || !rightAB)
        {
            Debug.LogError("Missing wheel ArticulationBody components");
            enabled = false;
            return;
        }

        // Initialize wheels
        ConfigureWheel(leftAB);
        ConfigureWheel(rightAB);

        // Setup ROS subscription
        var ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TwistMsg>(topicName, OnCmdVelReceived);
    }

    void FixedUpdate()
    {
        // Apply timeout
        if (lastCmdTime >= 0f && Time.fixedTime - lastCmdTime > timeout)
            command = Vector2.zero;

        // Differential drive kinematics
        var halfTrack = trackWidth * 0.5f;
        var leftSpeed = Mathf.Clamp(command.x - command.y * halfTrack, -maxSpeed, maxSpeed);
        var rightSpeed = Mathf.Clamp(command.x + command.y * halfTrack, -maxSpeed, maxSpeed);

        // Apply wheel speeds (convert to deg/s)
        SetWheelSpeed(leftAB, leftSpeed / wheelRadius * Mathf.Rad2Deg);
        SetWheelSpeed(rightAB, rightSpeed / wheelRadius * Mathf.Rad2Deg);
    }

    void OnCmdVelReceived(TwistMsg msg)
    {
        // Extract linear (forward/backward) and angular (rotation) velocities
        float linear = (float)msg.linear.x;
        float angular = (float)msg.angular.z;
        
        command = new Vector2(linear, angular);
        lastCmdTime = Time.fixedTime;
    }

    void ConfigureWheel(ArticulationBody ab)
    {
        var drive = ab.xDrive;
        drive.driveType = ArticulationDriveType.Velocity;
        drive.forceLimit = forceLimit;
        drive.damping = damping;
        ab.xDrive = drive;
    }

    void SetWheelSpeed(ArticulationBody ab, float degPerSec)
    {
        var drive = ab.xDrive;
        drive.targetVelocity = degPerSec;
        ab.xDrive = drive;
    }
}