using System.Collections.Generic;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;
using Unity.Robotics.Core;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

public class LidarSensor : MonoBehaviour
{
    [Header("Lidar Configuration")]
    public GameObject frameIdObject;
    public float scanRate = 10f; // Hz
    public float maxRange = 10f;
    public float minRange = 0.1f;
    
    [Header("Scan Range Settings")]
    [Tooltip("Start angle in degrees (-90 = right side)")]
    public float startAngleDegrees = -90f;
    [Tooltip("End angle in degrees (+90 = left side)")]  
    public float endAngleDegrees = 90f;
    [Tooltip("Number of rays across the angular range")]
    public int numberOfRays = 180;
    
    [Header("Visualization")]
    public bool showRays = true;
    public Color rayColor = Color.cyan;
    
    private ROSConnection ros;
    private double nextScanTime;
    private List<Vector3> hitPoints = new List<Vector3>();

    void Start()
    {
        if (frameIdObject == null)
            frameIdObject = gameObject;
            
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<LaserScanMsg>("/scan");
        
        // Initialize next scan time
        nextScanTime = Clock.Now + (1.0 / scanRate);
        
        Debug.Log($"Simple Lidar initialized: {numberOfRays} rays, {maxRange}m range, {scanRate}Hz");
    }

    void Update()
    {
        // Check if it's time for next scan using ROS time
        if (Clock.Now < nextScanTime)
            return;
            
        PerformScan();
        
        // Schedule next scan
        nextScanTime += (1.0 / scanRate);
    }

    void PerformScan()
    {
        Vector3 origin = frameIdObject.transform.position;
        float baseYaw = frameIdObject.transform.eulerAngles.y;
        
        List<float> ranges = new List<float>();
        hitPoints.Clear();
        
        // Calculate angular range
        float totalAngleRange = endAngleDegrees - startAngleDegrees;
        
        // Perform scan across the specified angle range
        for (int i = 0; i < numberOfRays; i++)
        {
            // FIXED: Calculate angle for proper ROS coordinate frame
            // ROS convention: 0° = forward, positive angles = counterclockwise (left)
            // For standard 180° LIDAR: -90° (right) to +90° (left)
            float rosAngle = startAngleDegrees + (totalAngleRange / (numberOfRays - 1)) * i;
            
            // Convert ROS angle to Unity world angle
            // Unity uses clockwise rotation, so negate the ROS angle
            float unityAngle = -rosAngle;
            float worldAngle = baseYaw + unityAngle;
            
            // Create ray direction
            Vector3 direction = Quaternion.Euler(0, worldAngle, 0) * Vector3.forward;
            Ray ray = new Ray(origin, direction);
            
            // Perform raycast
            if (Physics.Raycast(ray, out RaycastHit hit, maxRange))
            {
                float distance = hit.distance;
                if (distance >= minRange)
                {
                    ranges.Add(distance);
                    hitPoints.Add(hit.point);
                }
                else
                {
                    ranges.Add(float.PositiveInfinity);
                    hitPoints.Add(origin + direction * maxRange);
                }
            }
            else
            {
                ranges.Add(float.PositiveInfinity);
                hitPoints.Add(origin + direction * maxRange);
            }
        }
        
        // Publish to ROS
        PublishScan(ranges);
    }

    void PublishScan(List<float> ranges)
    {
        var timestamp = new TimeStamp(Clock.time);
        
        // Convert angles to radians
        float startAngleRad = startAngleDegrees * Mathf.Deg2Rad;
        float endAngleRad = endAngleDegrees * Mathf.Deg2Rad;
        float totalAngleRange = endAngleRad - startAngleRad;
        
        var scanMsg = new LaserScanMsg
        {
            header = new HeaderMsg
            {
                frame_id = frameIdObject.name,
                stamp = new TimeMsg
                {
                    sec = timestamp.Seconds,
                    nanosec = timestamp.NanoSeconds
                }
            },
            angle_min = startAngleRad,                        // Start angle in radians
            angle_max = endAngleRad,                          // End angle in radians
            angle_increment = totalAngleRange / (numberOfRays - 1), // Angular step
            time_increment = 0f,                              // Instant capture
            scan_time = 1f / scanRate,                        // Time between scans
            range_min = minRange,
            range_max = maxRange,
            ranges = ranges.ToArray(),
            intensities = new float[ranges.Count]             // Empty array (no intensity data)
        };
        
        ros.Publish("/scan", scanMsg);
    }

    void OnDrawGizmos()
    {
        if (!showRays || frameIdObject == null) return;
        
        Vector3 origin = frameIdObject.transform.position;
        Gizmos.color = rayColor;
        
        // Draw scan rays
        for (int i = 0; i < hitPoints.Count; i++)
        {
            Gizmos.DrawLine(origin, hitPoints[i]);
        }
        
        // Draw scan arc outline using proper ROS coordinate frame
        float baseYaw = frameIdObject.transform.eulerAngles.y;
        float totalAngleRange = endAngleDegrees - startAngleDegrees;
        
        // FIXED: Use proper coordinate conversion for visualization
        float startUnityAngle = -startAngleDegrees;  // Convert ROS to Unity angle
        Vector3 lastPoint = origin + Quaternion.Euler(0, baseYaw + startUnityAngle, 0) * Vector3.forward * maxRange;
        
        int arcSegments = Mathf.Max(10, numberOfRays / 10); // At least 10 segments for smooth arc
        for (int i = 1; i <= arcSegments; i++)
        {
            float rosAngle = startAngleDegrees + (totalAngleRange / arcSegments) * i;
            float unityAngle = -rosAngle;  // Convert ROS to Unity angle
            Vector3 direction = Quaternion.Euler(0, baseYaw + unityAngle, 0) * Vector3.forward;
            Vector3 point = origin + direction * maxRange;
            Gizmos.DrawLine(lastPoint, point);
            lastPoint = point;
        }
    }
}