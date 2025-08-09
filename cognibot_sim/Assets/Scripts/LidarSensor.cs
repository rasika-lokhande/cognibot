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
    public int numberOfRays = 360; // 1 ray per degree
    
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
        
        // Perform all raycasts instantly
        for (int i = 0; i < numberOfRays; i++)
        {
            // Calculate angle for this ray (0 to 360 degrees)
            // ROS convention: 0° = forward (X-axis), counterclockwise positive
            // Unity uses left-handed, ROS uses right-handed - negate angle for correct orientation
            float angle = -(360f / numberOfRays) * i;
            float worldAngle = baseYaw + angle;
            
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
            angle_min = 0f,                                    // Start at 0° (forward)
            angle_max = 2f * Mathf.PI,                        // Full 360° in radians
            angle_increment = (2f * Mathf.PI) / numberOfRays, // Angular step
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
        
        // Draw scan circle outline
        float baseYaw = frameIdObject.transform.eulerAngles.y;
        Vector3 lastPoint = origin + Quaternion.Euler(0, baseYaw, 0) * Vector3.forward * maxRange;
        
        for (int i = 1; i <= numberOfRays; i++)
        {
            float angle = -(360f / numberOfRays) * i;  // Negate for ROS coordinate system
            Vector3 direction = Quaternion.Euler(0, baseYaw + angle, 0) * Vector3.forward;
            Vector3 point = origin + direction * maxRange;
            Gizmos.DrawLine(lastPoint, point);
            lastPoint = point;
        }
    }
}