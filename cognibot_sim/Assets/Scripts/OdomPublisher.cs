using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.Core;
using RosMessageTypes.Nav;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using Unity.Robotics.UrdfImporter;

[RequireComponent(typeof(AGVController))]
public class OdometryPublisher : MonoBehaviour
{
    [Header("Publishing Settings")]
    [SerializeField] private float publishInterval = 0.02f; // seconds (0.02s = 50Hz)
    [SerializeField] private bool useFixedTimeStep = true; // true = time-based, false = frame-based
    [SerializeField] private string odomTopic = "/odom";
    [SerializeField] private string odomFrame = "odom";
    [SerializeField] private string baseFrame = "base_link";
    
    [Header("Covariance Settings - Simulation")]
    [Tooltip("Position uncertainty in meters² (very low for perfect simulation)")]
    [SerializeField] private float positionCovariance = 0.001f; // 1mm uncertainty
    [Tooltip("Orientation uncertainty in radians² (very low for perfect simulation)")]
    [SerializeField] private float orientationCovariance = 0.0001f; // ~0.6° uncertainty
    [Tooltip("Velocity uncertainty in (m/s)² and (rad/s)²")]
    [SerializeField] private float velocityCovariance = 0.01f; // Small velocity noise
    
    [Header("Advanced Covariance (Optional)")]
    [SerializeField] private bool useRealisticNoise = false;
    [Tooltip("Simulates real-world wheel slip, encoder noise, etc.")]
    [SerializeField] private float wheelSlipFactor = 0.02f; // 2% slip
    [SerializeField] private float encoderNoise = 0.001f; // Encoder uncertainty
    
    // Reference to AGV controller (no duplication!)
    private AGVController agvController;
    
    // Robot state
    private Vector3 robotPosition = Vector3.zero;
    private float robotYaw = 0f;
    private Vector3 robotVelocity = Vector3.zero;
    private float robotAngularVelocity = 0f;
    
    // Timing
    private float lastPublishTime = 0f;
    private float previousTime;
    
    // ROS
    private ROSConnection ros;
    
    void Start()
    {
        // Get reference to AGV controller (reuse its configuration)
        agvController = GetComponent<AGVController>();
        if (agvController == null)
        {
            Debug.LogError($"{name}: AGVController component required!");
            return;
        }
        
        InitializeROS();
        previousTime = Time.fixedTime;
        
        Debug.Log($"{name}: Using Unity.Robotics.Core.Clock for sim_time consistency");
        Debug.Log($"{name}: Clock mode: {Clock.Mode}");
    }
    
    void InitializeROS()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<OdometryMsg>(odomTopic);
        Debug.Log($"{name}: Publishing odometry to {odomTopic}");
    }
    
    void FixedUpdate()
    {
        if (agvController == null) return;
        
        float currentTime = Time.fixedTime;
        float deltaTime = currentTime - previousTime;
        
        if (deltaTime <= 0) return;
        
        // Always update odometry calculation
        UpdateOdometry(deltaTime);
        
        // Publish based on selected timing mode
        if (useFixedTimeStep)
        {
            // Time-based publishing (consistent intervals regardless of framerate)
            if (currentTime - lastPublishTime >= publishInterval)
            {
                PublishOdometry();
                lastPublishTime = currentTime;
            }
        }
        else
        {
            // Frame-based publishing (legacy mode for comparison)
            // This would publish every N frames instead
            PublishOdometry();
        }
        
        previousTime = currentTime;
    }
    
    void UpdateOdometry(float deltaTime)
    {
        // Get wheel velocities from AGV controller's wheels (reuse existing components!)
        float leftWheelVel = GetWheelVelocity(GetLeftWheelAB());
        float rightWheelVel = GetWheelVelocity(GetRightWheelAB());
        
        // Calculate robot motion using AGV controller's parameters
        CalculateRobotMotion(leftWheelVel, rightWheelVel, deltaTime);
    }
    
    void CalculateRobotMotion(float leftWheelVel, float rightWheelVel, float deltaTime)
    {
        // Use AGV controller's parameters (no duplication!)
        float wheelRadius = agvController.wheelRadius;
        float trackWidth = agvController.trackWidth;
        
        // Convert wheel angular velocities to linear velocities
        float leftLinearVel = leftWheelVel * wheelRadius;
        float rightLinearVel = rightWheelVel * wheelRadius;
        
        // Differential drive kinematics
        float linearVelocity = (leftLinearVel + rightLinearVel) / 2.0f;
        float angularVelocity = (rightLinearVel - leftLinearVel) / trackWidth;
        
        // Update robot pose
        float deltaDistance = linearVelocity * deltaTime;
        float deltaTheta = angularVelocity * deltaTime;
        
        robotPosition.x += deltaDistance * Mathf.Cos(robotYaw + deltaTheta / 2.0f);
        robotPosition.z += deltaDistance * Mathf.Sin(robotYaw + deltaTheta / 2.0f);
        robotYaw += deltaTheta;
        
        // Normalize yaw to [-π, π]
        robotYaw = Mathf.Atan2(Mathf.Sin(robotYaw), Mathf.Cos(robotYaw));
        
        // Store velocities
        robotVelocity = new Vector3(
            linearVelocity * Mathf.Cos(robotYaw),
            0,
            linearVelocity * Mathf.Sin(robotYaw)
        );
        robotAngularVelocity = angularVelocity;
    }
    
    // Helper methods to access AGV controller's private fields
    ArticulationBody GetLeftWheelAB()
    {
        return agvController.leftWheel?.GetComponent<ArticulationBody>();
    }
    
    ArticulationBody GetRightWheelAB()
    {
        return agvController.rightWheel?.GetComponent<ArticulationBody>();
    }
    
    float GetWheelVelocity(ArticulationBody ab)
    {
        if (ab == null) return 0f;
        return ab.jointVelocity[0] * Mathf.Deg2Rad;
    }
    
    void PublishOdometry()
    {
        var odomMsg = new OdometryMsg
        {
            header = new HeaderMsg
            {
                // CRITICAL: Use Unity.Robotics.Core.Clock for sim_time consistency
                stamp = new TimeStamp(Clock.time), // Matches /clock topic exactly
                frame_id = odomFrame
            },
            child_frame_id = baseFrame,
            
            pose = new PoseWithCovarianceMsg
            {
                pose = new PoseMsg
                {
                    position = robotPosition.To<FLU>(),
                    orientation = Quaternion.Euler(0, robotYaw * Mathf.Rad2Deg, 0).To<FLU>()
                },
                covariance = CreatePoseCovariance()
            },
            
            twist = new TwistWithCovarianceMsg
            {
                twist = new TwistMsg
                {
                    linear = robotVelocity.To<FLU>(),
                    angular = new Vector3Msg(0, robotAngularVelocity, 0)
                },
                covariance = CreateTwistCovariance()
            }
        };
        
        ros.Publish(odomTopic, odomMsg);
    }
    
    double[] CreatePoseCovariance()
    {
        var covariance = new double[36];
        
        // Base covariance values
        float posVar = positionCovariance;
        float oriVar = orientationCovariance;
        
        // Add realistic noise if enabled
        if (useRealisticNoise)
        {
            // Increase uncertainty based on velocity (higher speed = more uncertainty)
            float speedFactor = 1.0f + robotVelocity.magnitude * wheelSlipFactor;
            posVar *= speedFactor;
            oriVar *= speedFactor;
        }
        
        // Position covariance (x, y, z) - diagonal elements
        covariance[0] = posVar;   // x-x variance
        covariance[7] = posVar;   // y-y variance  
        covariance[14] = posVar * 0.1; // z-z variance (much lower for ground robots)
        
        // Orientation covariance (roll, pitch, yaw) - diagonal elements
        covariance[21] = oriVar * 0.1; // roll-roll (lower for ground robots)
        covariance[28] = oriVar * 0.1; // pitch-pitch (lower for ground robots)
        covariance[35] = oriVar;        // yaw-yaw (main uncertainty for differential drive)
        
        // Cross-correlations (typically small for odometry)
        // Example: x-y correlation due to turning
        if (Mathf.Abs(robotAngularVelocity) > 0.1f)
        {
            covariance[1] = covariance[6] = posVar * 0.1f; // Small x-y correlation when turning
        }
        
        return covariance;
    }
    
    double[] CreateTwistCovariance()
    {
        var covariance = new double[36];
        
        float velVar = velocityCovariance;
        
        // Add encoder noise simulation
        if (useRealisticNoise)
        {
            velVar += encoderNoise;
        }
        
        // Linear velocity covariance
        covariance[0] = velVar;         // vx-vx
        covariance[7] = velVar * 0.5f;  // vy-vy (lower for differential drive)
        covariance[14] = velVar * 0.1f; // vz-vz (very low for ground robots)
        
        // Angular velocity covariance  
        covariance[21] = velVar * 0.1f; // wx-wx (low for ground robots)
        covariance[28] = velVar * 0.1f; // wy-wy (low for ground robots)
        covariance[35] = velVar;        // wz-wz (main uncertainty for yaw rate)
        
        return covariance;
    }
    
    // Public interface
    public Vector3 GetCurrentPosition() => robotPosition;
    public float GetCurrentYaw() => robotYaw;
    public Vector3 GetCurrentVelocity() => robotVelocity;
    public float GetCurrentAngularVelocity() => robotAngularVelocity;
    
    public void ResetOdometry()
    {
        robotPosition = Vector3.zero;
        robotYaw = 0f;
        robotVelocity = Vector3.zero;
        robotAngularVelocity = 0f;
    }
    
    public void SetInitialPose(Vector3 position, float yaw)
    {
        robotPosition = position;
        robotYaw = yaw;
    }
    
    void OnDrawGizmos()
    {
        if (Application.isPlaying)
        {
            Gizmos.color = Color.blue;
            Gizmos.DrawWireSphere(robotPosition, 0.1f);
            
            Gizmos.color = Color.red;
            Vector3 forward = new Vector3(Mathf.Cos(robotYaw), 0, Mathf.Sin(robotYaw)) * 0.3f;
            Gizmos.DrawRay(robotPosition, forward);
            
            if (robotVelocity.magnitude > 0.01f)
            {
                Gizmos.color = Color.green;
                Gizmos.DrawRay(robotPosition, robotVelocity * 0.5f);
            }
        }
    }
}