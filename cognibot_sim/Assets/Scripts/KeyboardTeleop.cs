using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class KeyboardTeleop : MonoBehaviour
{
    [SerializeField] private float linearSpeed = 0.5f;
    [SerializeField] private float angularSpeed = 1.0f;
    [SerializeField] private string topicName = "/cmd_vel";

    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TwistMsg>(topicName);
    }

    void Update()
    {
        float linear = Input.GetAxisRaw("Vertical") * linearSpeed;
        float angular = -Input.GetAxisRaw("Horizontal") * angularSpeed;

        if (linear != 0 || angular != 0)
        {
            ros.Publish(topicName, new TwistMsg
            {
                linear = new Vector3Msg(linear, 0, 0),
                angular = new Vector3Msg(0, 0, angular)
            });
        }
    }
}