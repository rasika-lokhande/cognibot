using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;
using System;

public class LaserScanPublisher : MonoBehaviour
{
    [Header("ROS Settings")]
    public string topicName = "/scan";

    [Header("Sensor Origin")]
    public GameObject lidarOrigin;  // Assign the camera or laser frame

    [Header("Scan Parameters")]
    public int numRays = 360;
    public float angleMin = -Mathf.PI;
    public float angleMax = Mathf.PI;
    public float rangeMin = 0.1f;
    public float rangeMax = 10.0f;
    public float scanFrequency = 10f; // Hz

    private float scanTimer = 0f;
    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<LaserScanMsg>(topicName);

        if (lidarOrigin == null)
            Debug.LogError("Lidar origin GameObject not assigned!");
    }

    void Update()
    {
        scanTimer += Time.deltaTime;
        if (scanTimer >= 1f / scanFrequency)
        {
            PublishLaserScan();
            scanTimer = 0f;
        }
    }

    void PublishLaserScan()
    {
        if (lidarOrigin == null) return;

        Vector3 origin = lidarOrigin.transform.position;
        Vector3 forward = lidarOrigin.transform.forward; // Z+ direction

        float[] ranges = new float[numRays];
        float[] intensities = new float[numRays];
        float angleIncrement = (angleMax - angleMin) / (numRays - 1);

        for (int i = 0; i < numRays; i++)
        {
            float angle = angleMin + i * angleIncrement;

            // Rotate in local XZ plane (around global Y axis)
            Vector3 direction = Quaternion.Euler(0, Mathf.Rad2Deg * angle, 0) * lidarOrigin.transform.forward;

            if (Physics.Raycast(origin, direction, out RaycastHit hit, rangeMax))
            {
                ranges[i] = hit.distance;
                intensities[i] = 1.0f;

                // Visual Debug: Green line for hits
                Debug.DrawRay(origin, direction * hit.distance, Color.green, 0.1f);
            }
            else
            {
                ranges[i] = rangeMax;
                intensities[i] = 0.0f;

                // Visual Debug: Red line for misses
                Debug.DrawRay(origin, direction * rangeMax, Color.red, 0.1f);
            }
        }

        // ROS Time
        TimeMsg time = new TimeMsg
        {
            sec = Mathf.FloorToInt(Time.time),
            nanosec = (uint)((Time.time - Mathf.Floor(Time.time)) * 1e9)
        };

        HeaderMsg header = new HeaderMsg
        {
            stamp = time,
            frame_id = "camera_link" // Use whatever TF frame you're using in ROS2
        };

        LaserScanMsg scanMsg = new LaserScanMsg
        {
            header = header,
            angle_min = angleMin,
            angle_max = angleMax,
            angle_increment = angleIncrement,
            time_increment = 1.0f / (scanFrequency * numRays),
            scan_time = 1.0f / scanFrequency,
            range_min = rangeMin,
            range_max = rangeMax,
            ranges = ranges,
            intensities = intensities
        };

        ros.Publish(topicName, scanMsg);
    }
}
