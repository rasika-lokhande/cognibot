using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class KeyboardTeleop : MonoBehaviour
{
    public float linearSpeed = 0.5f;
    public float angularSpeed = 1.0f;

    private ROSConnection ros;

    [SerializeField]
    private string topicName = "/cmd_vel";

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TwistMsg>(topicName);
    }

    void Update()
    {
        float linear = 0f;
        float angular = 0f;

        if (Input.GetKey(KeyCode.W))
            linear = linearSpeed;
        if (Input.GetKey(KeyCode.S))
            linear = -linearSpeed;
        if (Input.GetKey(KeyCode.A))
            angular = angularSpeed;
        if (Input.GetKey(KeyCode.D))
            angular = -angularSpeed;

        TwistMsg twist = new TwistMsg
        {
            linear = new Vector3Msg(linear, 0, 0),
            angular = new Vector3Msg(0, 0, angular)
        };

        ros.Publish(topicName, twist);
    }
}
