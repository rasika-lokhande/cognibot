using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;
using System;
using System.Collections.Generic;

public class PointCloudPublisher3D : MonoBehaviour
{
    [Header("ROS Settings")]
    public string topicName = "/point_cloud";

    [Header("Scan Settings")]
    public GameObject lidarOrigin;
    public int verticalRays = 16;
    public int horizontalRays = 360;
    public float verticalFOVMin = -15f;
    public float verticalFOVMax = 15f;
    public float rangeMax = 10f;

    [Header("Timing")]
    public float publishRate = 10f; // Hz
    private float publishTimer = 0f;

    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PointCloud2Msg>(topicName);
    }

    void Update()
    {
        publishTimer += Time.deltaTime;
        if (publishTimer >= 1.0f / publishRate)
        {
            PublishPointCloud();
            publishTimer = 0f;
        }
    }

    void PublishPointCloud()
    {
        if (lidarOrigin == null) return;

        List<float> pointData = new List<float>();
        Transform tf = lidarOrigin.transform;

        float vStep = (verticalFOVMax - verticalFOVMin) / Mathf.Max(1, verticalRays - 1);
        float hStep = 360f / Mathf.Max(1, horizontalRays);

        for (int v = 0; v < verticalRays; v++)
        {
            float pitch = verticalFOVMin + v * vStep;

            for (int h = 0; h < horizontalRays; h++)
            {
                float yaw = h * hStep;

                Quaternion rot = tf.rotation * Quaternion.Euler(pitch, yaw, 0);
                Vector3 dir = rot * Vector3.forward;
                Vector3 origin = tf.position;

                if (Physics.Raycast(origin, dir, out RaycastHit hit, rangeMax))
                {
                    Vector3 point = hit.point;
                    pointData.Add(point.x);
                    pointData.Add(point.y);
                    pointData.Add(point.z);
                }
            }
        }

        int pointCount = pointData.Count / 3;

        // Build PointCloud2 fields (x, y, z = 3 floats)
        PointFieldMsg[] fields = new PointFieldMsg[3];
        fields[0] = new PointFieldMsg("x", 0, PointFieldMsg.FLOAT32, 1);
        fields[1] = new PointFieldMsg("y", 4, PointFieldMsg.FLOAT32, 1);
        fields[2] = new PointFieldMsg("z", 8, PointFieldMsg.FLOAT32, 1);

        // Flatten data to byte array
        byte[] data = new byte[pointCount * 12];
        for (int i = 0; i < pointCount; i++)
        {
            Buffer.BlockCopy(BitConverter.GetBytes(pointData[i * 3]), 0, data, i * 12 + 0, 4);
            Buffer.BlockCopy(BitConverter.GetBytes(pointData[i * 3 + 1]), 0, data, i * 12 + 4, 4);
            Buffer.BlockCopy(BitConverter.GetBytes(pointData[i * 3 + 2]), 0, data, i * 12 + 8, 4);
        }

        // Header
        TimeMsg time = new TimeMsg
        {
             sec = Mathf.FloorToInt(Time.time),
            nanosec = (uint)((Time.time - Mathf.Floor(Time.time)) * 1e9)
        };

        HeaderMsg header = new HeaderMsg
        {
            stamp = time,
            frame_id = "camera_link" // or "lidar_link"
        };

        PointCloud2Msg cloud = new PointCloud2Msg
        {
            header = header,
            height = 1,
            width = (uint)pointCount,
            is_bigendian = false,
            is_dense = false,
            point_step = 12,
            row_step = (uint)(12 * pointCount),
            fields = fields,
            data = data
        };

        ros.Publish(topicName, cloud);
    }
}
