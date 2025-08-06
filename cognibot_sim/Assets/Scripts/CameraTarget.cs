
using UnityEngine;
using System;
using System.Collections.Generic;
public class CameraTarget : MonoBehaviour
{
    public Transform robotBase;

    void Start()
    {
        if (robotBase)
        {
            transform.position = robotBase.position;
            transform.rotation = robotBase.rotation;
        }

        else
        {
            Debug.LogError("Attach camera target");
        }
    }

    void Update()
    {
        if (robotBase)
            transform.position = robotBase.position;
            transform.rotation = robotBase.rotation;
    }
}
