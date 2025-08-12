using UnityEngine;

public class BaseFootprintFollower : MonoBehaviour
{
    [SerializeField] private Transform baseLink;
    
    void Update()
    {
        if (baseLink)
        {
            // Keep same X,Z position but project to ground (Y=0)
            transform.position = new Vector3(baseLink.position.x, 0, baseLink.position.z);
            transform.rotation = Quaternion.Euler(0, baseLink.eulerAngles.y, 0);
        }
    }
}