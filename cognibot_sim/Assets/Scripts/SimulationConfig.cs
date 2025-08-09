using UnityEngine;

/// <summary>
/// Configures Unity for consistent simulation performance:
/// - Locks frame rate
/// - Disables VSync
/// - Sets physics timestep
/// - Optionally displays real-time FPS in top-right corner
/// </summary>
public class SimulationPerformanceConfig : MonoBehaviour
{
    [Header("Frame Settings")]
    public int targetFrameRate = 60;

    [Header("Physics Settings")]
    public float fixedTimestep = 0.02f;         // 50 Hz
    public float maxAllowedTimestep = 0.05f;    // Cap max physics step time

    [Header("Debug")]
    public bool showFPS = true;

    private GUIStyle fpsStyle;

    private void Awake()
    {
        // Disable VSync
        QualitySettings.vSyncCount = 0;

        // Set frame rate
        Application.targetFrameRate = targetFrameRate;

        // Physics time settings
        Time.fixedDeltaTime = fixedTimestep;
        Time.maximumDeltaTime = maxAllowedTimestep;

        // Configure FPS display style
        fpsStyle = new GUIStyle
        {
            fontSize = 16,
            normal = new GUIStyleState { textColor = Color.white },
            alignment = TextAnchor.UpperRight
        };

        Debug.Log($"[SimulationPerformanceConfig] TargetFrameRate: {targetFrameRate}, FixedTimestep: {fixedTimestep}, VSync: OFF");
    }

    private void OnGUI()
    {
        if (!showFPS) return;

        float fps = 1.0f / Time.deltaTime;
        Rect rect = new Rect(Screen.width - 110, 10, 100, 30);
        GUI.Label(rect, $"FPS: {fps:F1}", fpsStyle);
    }
}
