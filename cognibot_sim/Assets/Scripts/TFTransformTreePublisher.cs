using System.Collections.Generic;
using System.Linq;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using RosMessageTypes.Tf2;
using Unity.Robotics.Core;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

/// <summary>
/// Publishes transform tree for robot links. Handles base_footprint -> base_link relationship.
/// </summary>
public class ROSTransformTreePublisher : MonoBehaviour
{
    [Header("Publishing Settings")]
    [SerializeField, Tooltip("Transform publishing rate in Hz")]
    private float publishRate = 20f;
    
    [SerializeField, Tooltip("ROS topic for transform messages")]
    private string tfTopic = "/tf";
    
    [Header("Frame Configuration")]
    [SerializeField, Tooltip("Enable map and odom frames for navigation")]
    private bool useGlobalFrames = false;
    
    [SerializeField, Tooltip("Global frame IDs (map, odom)")]
    private List<string> globalFrameIds = new List<string> { "map", "odom" };
    
    [SerializeField, Tooltip("Root GameObject (use base_footprint for proper robot setup)")]
    private GameObject rootGameObject;

    private double lastPublishTime;
    private TransformTreeNode transformRoot;
    private ROSConnection ros;

    private double PublishPeriod => 1.0 / publishRate;
    private bool ShouldPublish => Clock.NowTimeInSeconds > lastPublishTime + PublishPeriod;

    void Start()
    {
        InitializeRoot();
        InitializeROS();
    }

    void Update()
    {
        if (ShouldPublish)
            PublishTransforms();
    }

    private void InitializeRoot()
    {
        if (!rootGameObject)
        {
            // Auto-find base_footprint or fallback to this GameObject
            rootGameObject = FindRootGameObject();
            Debug.Log($"Using {rootGameObject.name} as transform root");
        }
        
        transformRoot = new TransformTreeNode(rootGameObject);
    }

    private GameObject FindRootGameObject()
    {
        // Look for base_footprint first (proper robot convention)
        var baseFootprint = GameObject.Find("base_footprint");
        if (baseFootprint) return baseFootprint;
        
        // Fallback to base_link
        var baseLink = GameObject.Find("base_link");
        if (baseLink) return baseLink;
        
        // Last resort - use this GameObject
        Debug.LogWarning("Could not find base_footprint or base_link, using current GameObject");
        return gameObject;
    }

    private void InitializeROS()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TFMessageMsg>(tfTopic);
        lastPublishTime = Clock.time + PublishPeriod;
    }

    private void PublishTransforms()
    {
        var transformList = new List<TransformStampedMsg>();
        
        // Add global frames if enabled
        if (useGlobalFrames && globalFrameIds.Count > 0)
        {
            AddGlobalFrameTransforms(transformList);
        }
        
        // Add all child transforms
        PopulateChildTransforms(transformList, transformRoot);
        
        // Publish the complete transform tree
        var tfMessage = new TFMessageMsg(transformList.ToArray());
        ros.Publish(tfTopic, tfMessage);
        lastPublishTime = Clock.FrameStartTimeInSeconds;
    }

    private void AddGlobalFrameTransforms(List<TransformStampedMsg> transformList)
    {
        // Root to global frame (e.g., base_footprint -> odom)
        var rootToGlobal = new TransformStampedMsg(
            new HeaderMsg(new TimeStamp(Clock.time), globalFrameIds.Last()),
            transformRoot.name,
            transformRoot.Transform.To<FLU>()
        );
        transformList.Add(rootToGlobal);
        
        // Chain global frames (e.g., odom -> map)
        for (int i = 1; i < globalFrameIds.Count; i++)
        {
            var globalChain = new TransformStampedMsg(
                new HeaderMsg(new TimeStamp(Clock.time), globalFrameIds[i - 1]),
                globalFrameIds[i],
                new TransformMsg() // Identity transform
            );
            transformList.Add(globalChain);
        }
    }

    private static void PopulateChildTransforms(List<TransformStampedMsg> transformList, TransformTreeNode node)
    {
        foreach (var child in node.Children)
        {
            transformList.Add(TransformTreeNode.ToTransformStamped(child));
            
            if (!child.IsALeafNode)
                PopulateChildTransforms(transformList, child);
        }
    }

    [ContextMenu("Find Base Footprint")]
    private void FindBaseFootprint()
    {
        rootGameObject = FindRootGameObject();
        Debug.Log($"Set root to: {rootGameObject.name}");
    }
}