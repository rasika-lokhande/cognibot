using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.Core;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;
using System.Collections.Generic;

[System.Serializable]
public class JointEntry
{
    [Tooltip("Exact joint name as defined in URDF")]
    public string jointName;
    
    [Tooltip("ArticulationBody for this joint")]
    public ArticulationBody jointBody;
}

/// <summary>
/// Publishes joint states for articulation joints
/// </summary>
public class JointStatePublisher : MonoBehaviour
{
    [SerializeField, Tooltip("ROS topic name")]
    private string topicName = "/joint_states";
    
    [SerializeField, Tooltip("Publish frequency in Hz")]
    private float publishFrequency = 10f;
    
    [SerializeField, Tooltip("Joint configuration")]
    private List<JointEntry> jointEntries;

    private ROSConnection ros;
    private double lastPublishTime;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<JointStateMsg>(topicName);
        
        if (jointEntries?.Count == 0)
            Debug.LogError("No joints assigned to JointStatePublisher!");
    }

    void Update()
    {
        if (Clock.time - lastPublishTime >= 1f / publishFrequency)
        {
            PublishJointStates();
            lastPublishTime = Clock.time;
        }
    }

    void PublishJointStates()
    {
        int count = jointEntries.Count;
        string[] names = new string[count];
        double[] positions = new double[count];
        double[] velocities = new double[count];
        double[] efforts = new double[count];

        for (int i = 0; i < count; i++)
        {
            var entry = jointEntries[i];
            names[i] = entry.jointName;
            positions[i] = entry.jointBody.jointPosition[0] * Mathf.Deg2Rad;
            velocities[i] = entry.jointBody.jointVelocity[0];
            efforts[i] = entry.jointBody.jointForce[0];
        }

        var msg = new JointStateMsg(
            new HeaderMsg 
            { 
                stamp = GetTimeMsg(), 
                frame_id = "" 
            },
            names, positions, velocities, efforts
        );

        ros.Publish(topicName, msg);
    }

    private TimeMsg GetTimeMsg()
    {
        double time = Clock.time;
        return new TimeMsg
        {
            sec = (int)time,
            nanosec = (uint)((time - (int)time) * 1e9)
        };
    }

#if UNITY_EDITOR
    [ContextMenu("Auto-Populate Joints")]
    public void AutoPopulateJoints()
    {
        jointEntries = new List<JointEntry>();
        foreach (var body in GetComponentsInChildren<ArticulationBody>())
        {
            if (body.jointType != ArticulationJointType.FixedJoint)
            {
                jointEntries.Add(new JointEntry
                {
                    jointName = body.name,
                    jointBody = body
                });
            }
        }
        Debug.Log($"Auto-populated {jointEntries.Count} joints");
    }
#endif
}