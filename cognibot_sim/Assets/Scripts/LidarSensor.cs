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
    [Tooltip("in Hz")]
    public float scanRate = 10f; 
    public float maxRange = 10f;
    public float minRange = 0.1f;
    
    [Header("Scan Range Settings")]
    [Range(-180f, 180f), Tooltip("Start angle in degrees (-90 = right side)")]
    public float startAngleDegrees = -90f;
    [Range(-180f, 180f), Tooltip("End angle in degrees (+90 = left side)")]  
    public float endAngleDegrees = 90f;
    [Range(10, 1080), Tooltip("Number of rays across the angular range")]
    public int numberOfRays = 180;
    
    [Header("Visualization")]
    public bool showRays = true;
    public Color rayColor = Color.cyan;
    
    private ROSConnection ros;
    private double nextScanTime;
    private readonly List<Vector3> hitPoints = new List<Vector3>();
    private readonly List<float> ranges = new List<float>();
    private float angleIncrement;
    private float totalAngleRange;

    void Start()
    {
        // Validate configuration
        if (!frameIdObject)
        {
            Debug.LogError("Frame ID GameObject must be assigned.", this);
            enabled = false;
            return;
        }
        
        if (endAngleDegrees <= startAngleDegrees)
        {
            Debug.LogError("End angle must be greater than start angle.", this);
            enabled = false;
            return;
        }
            
        // Initialize ROS
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<LaserScanMsg>("/scan");
        
        // Pre-calculate constants
        totalAngleRange = endAngleDegrees - startAngleDegrees;
        angleIncrement = totalAngleRange / (numberOfRays - 1);
        nextScanTime = Clock.Now + (1.0 / scanRate);
        
        Debug.Log($"LIDAR initialized: {numberOfRays} rays, {maxRange}m range, {scanRate}Hz");
    }

    void Update()
    {
        if (Clock.Now >= nextScanTime)
        {
            PerformScan();
            nextScanTime += (1.0 / scanRate);
        }
    }

    void PerformScan()
    {
        var origin = frameIdObject.transform.position + Vector3.up * 0.01f; // Small offset to avoid edge cases
        var baseYaw = frameIdObject.transform.eulerAngles.y;
        
        // Clear and pre-allocate capacity
        ranges.Clear();
        hitPoints.Clear();
        if (ranges.Capacity < numberOfRays)
        {
            ranges.Capacity = numberOfRays;
            hitPoints.Capacity = numberOfRays;
        }
        
        // Perform scan
        for (int i = 0; i < numberOfRays; i++)
        {
            // Convert ROS angle convention to Unity world angle
            float rosAngle = startAngleDegrees + angleIncrement * i;
            float worldAngle = baseYaw - rosAngle; // Unity uses clockwise, ROS counter-clockwise
            
            var direction = new Vector3(Mathf.Sin(worldAngle * Mathf.Deg2Rad), 0, 
                                      Mathf.Cos(worldAngle * Mathf.Deg2Rad));
            
            // Use RaycastAll to handle overlapping colliders properly
            var hits = Physics.RaycastAll(origin, direction, maxRange);
            float distance = float.PositiveInfinity;
            Vector3 hitPoint = origin + direction * maxRange;
            
            if (hits.Length > 0)
            {
                float closestDistance = float.MaxValue;
                foreach (var hit in hits)
                {
                    if (hit.distance >= minRange && hit.distance < closestDistance)
                    {
                        closestDistance = hit.distance;
                        hitPoint = hit.point;
                    }
                }
                if (closestDistance < float.MaxValue)
                    distance = closestDistance;
            }
            
            ranges.Add(distance);
            hitPoints.Add(hitPoint);
            
            // Draw ray synchronized with scan timing
            if (showRays)
                Debug.DrawRay(origin, direction * (float.IsPositiveInfinity(distance) ? maxRange : distance), 
                             rayColor, 1f / scanRate);
        }
        
        PublishScan();
    }

    void PublishScan()
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
            angle_min = startAngleDegrees * Mathf.Deg2Rad,
            angle_max = endAngleDegrees * Mathf.Deg2Rad,
            angle_increment = angleIncrement * Mathf.Deg2Rad,
            time_increment = 0f,
            scan_time = 1f / scanRate,
            range_min = minRange,
            range_max = maxRange,
            ranges = ranges.ToArray(),
            intensities = new float[ranges.Count]
        };
        
        ros.Publish("/scan", scanMsg);
    }

    void OnDrawGizmosSelected()
    {
        if (!showRays || !frameIdObject) return;
        
        var origin = frameIdObject.transform.position;
        Gizmos.color = rayColor;
        
        // Draw scan arc outline only (rays are now drawn with Debug.DrawRay)
        var baseYaw = frameIdObject.transform.eulerAngles.y;
        const int arcSegments = 20;
        
        var lastPoint = GetArcPoint(origin, baseYaw, startAngleDegrees);
        for (int i = 1; i <= arcSegments; i++)
        {
            float angle = startAngleDegrees + (totalAngleRange * i / arcSegments);
            var point = GetArcPoint(origin, baseYaw, angle);
            Gizmos.DrawLine(lastPoint, point);
            lastPoint = point;
        }
    }
    
    Vector3 GetArcPoint(Vector3 origin, float baseYaw, float rosAngle)
    {
        float worldAngle = baseYaw - rosAngle;
        return origin + new Vector3(Mathf.Sin(worldAngle * Mathf.Deg2Rad), 0, 
                                   Mathf.Cos(worldAngle * Mathf.Deg2Rad)) * maxRange;
    }
}