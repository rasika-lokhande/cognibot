using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;               // For JointStateMsg
using RosMessageTypes.Std;                  // For HeaderMsg
using RosMessageTypes.BuiltinInterfaces;    // For TimeMsg
//using RosMessageTypes.RosMessages;
using System;
using System.Collections.Generic;
using Unity.Robotics.UrdfImporter;


/// <summary>
/// Publishes joint states (position, velocity, effort) for a list of joints
/// </summary>
/// 

[System.Serializable]
public class JointEntry
{
    [Tooltip("Exact joint name as defined in the URDF")]
    public string jointName;

    [Tooltip("ArticulationBody corresponding to this joint")]
    public ArticulationBody jointBody;
}

public class JointStatePublisher : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "/joint_states";

    [Tooltip("List of joints and their URDF names")]
    public List<JointEntry> jointEntries;

    [Tooltip("Publish interval in seconds")]
    public float publishMessageFrequency = 0.5f;

    private float timeElapsed;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<JointStateMsg>(topicName);
       

        if (jointEntries == null || jointEntries.Count == 0)
        {
            Debug.LogError("JointStatePublisher: No joints assigned!");
        }
    }

#if UNITY_EDITOR
    public void EditorAutoPopulateJoints()
    {
        jointEntries = new List<JointEntry>();
        ArticulationBody[] bodies = GetComponentsInChildren<ArticulationBody>();

        foreach (var body in bodies)
        {
            if (body.jointType != ArticulationJointType.FixedJoint)
            {
                jointEntries.Add(new JointEntry
                {

                    jointBody = body
                });
            }
        }

        Debug.Log($"[Editor] Auto-populated {jointEntries.Count} joints.");
        

}
#endif



    void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed >= publishMessageFrequency)
        {
            PublishJointStates();
            timeElapsed = 0;
        }
    }

    void PublishJointStates()
    {
        //int count = jointBodies.Count;
        int count = jointEntries.Count;

        string[] names = new string[count];
        double[] positions = new double[count];
        double[] velocities = new double[count];
        double[] efforts = new double[count];

        for (int i = 0; i < count; i++)
        {
            var joint = jointEntries[i].jointBody;

            names[i] = jointEntries[i].jointName;
            positions[i] = Mathf.Deg2Rad * joint.jointPosition[0]; // Convert to radians
            velocities[i] = joint.jointVelocity[0];                 // Degrees/s
            efforts[i] = joint.jointForce[0];                       // Nm
            //Debug.Log($"Joint {names[i]}: pos={positions[i]}, vel={velocities[i]}, effort={efforts[i]}");
        }



       


        // Timestamp
        TimeMsg timeMsg = new TimeMsg
        {
            sec = Mathf.FloorToInt(Time.time),
            nanosec = (uint)((Time.time - Mathf.Floor(Time.time)) * 1e9)
        };


        HeaderMsg header = new HeaderMsg
        {
            stamp = timeMsg,
            frame_id = ""
        };

        // Compose the message
        JointStateMsg jointState = new JointStateMsg(
            header,
            names,
            positions,
            velocities,
            efforts
        );

        ros.Publish(topicName, jointState);

    }
}
