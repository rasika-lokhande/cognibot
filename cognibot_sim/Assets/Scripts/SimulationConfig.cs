using UnityEngine;

/// <summary>
/// Configures Unity for consistent simulation performance
/// </summary>
public class SimulationPerformanceConfig : MonoBehaviour
{
    [SerializeField] private int targetFrameRate = 60;
    [SerializeField] private float fixedTimestep = 0.02f;
    [SerializeField] private float maxTimestep = 0.05f;
    [SerializeField] private bool showFPS = true;

    void Awake()
    {
        QualitySettings.vSyncCount = 0;
        Application.targetFrameRate = targetFrameRate;
        Time.fixedDeltaTime = fixedTimestep;
        Time.maximumDeltaTime = maxTimestep;
    }

    void OnGUI()
    {
        if (showFPS)
            GUI.Label(new Rect(Screen.width - 100, 5, 95, 20), 
                     $"FPS: {1f / Time.deltaTime:F0}");
    }
}