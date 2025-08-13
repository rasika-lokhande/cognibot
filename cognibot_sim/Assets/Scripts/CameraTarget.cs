using UnityEngine;

public class CameraTarget : MonoBehaviour
{
    [SerializeField, Tooltip("Transform to follow with camera")]
    private Transform cameraTarget;

    void Start()
    {
        if (!cameraTarget)
        {
            Debug.LogError("Camera Target not assigned to CameraTarget");
            return;
        }
        
        SyncTransform();
    }

    void Update()
    {
        if (cameraTarget)
            SyncTransform();
    }

    private void SyncTransform()
    {
        transform.position = cameraTarget.position;
        transform.rotation = cameraTarget.rotation;
    }
}