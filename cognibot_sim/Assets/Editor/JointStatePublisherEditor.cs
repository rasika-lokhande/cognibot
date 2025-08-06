using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(JointStatePublisher))]
public class JointStatePublisherEditor : Editor
{
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        JointStatePublisher jsp = (JointStatePublisher)target;
        if (GUILayout.Button("Auto-Populate Joints"))
        {
            jsp.EditorAutoPopulateJoints();
            EditorUtility.SetDirty(jsp); // Mark the object as dirty so Unity saves it
        }
    }
}
