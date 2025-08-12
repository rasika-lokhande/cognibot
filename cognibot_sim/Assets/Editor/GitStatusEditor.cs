using UnityEngine;
using UnityEditor;
using System.Diagnostics;
using System.IO;
using UnityEngine.UIElements;

/// <summary>
/// Displays Git branch information in Unity's status bar using modern Unity APIs
/// Place this script in an "Editor" folder in your project
/// Compatible with Unity 2019.1+ using UIElements and EditorGUILayout approaches
/// </summary>
[InitializeOnLoad]
public static class GitStatusBar
{
    private static string cachedBranchInfo = "";
    private static double lastUpdateTime = 0;
    private static double updateInterval = 3.0;
    private static bool isGitRepository = true;
    private static GUIStyle statusBarStyle;
    private static string gitRepositoryRoot; // Store the actual Git repository root path
    private static string[] changedFiles = new string[0]; // Store list of changed files
    private static bool showDetailedFiles = false; // Toggle for showing file details
    
    static GitStatusBar()
    {
        EditorApplication.update += UpdateGitInformation;
        SceneView.duringSceneGui += OnSceneGUI;
        
        // Initialize our custom style for the status bar
        InitializeStatusBarStyle();
        
        // Initial update
        UpdateGitInformation();
    }
    
    /// <summary>
    /// Initialize the visual style for our status bar display
    /// This creates a consistent look that fits with Unity's design
    /// </summary>
    static void InitializeStatusBarStyle()
    {
        statusBarStyle = new GUIStyle();
        statusBarStyle.normal.textColor = EditorGUIUtility.isProSkin ? Color.white : Color.black;
        statusBarStyle.fontSize = 11;
        statusBarStyle.alignment = TextAnchor.MiddleLeft;
        statusBarStyle.padding = new RectOffset(5, 5, 2, 2);
    }
    
    /// <summary>
    /// This creates a persistent status bar at the bottom of Unity
    /// We use SceneView overlay as a reliable way to show persistent information
    /// </summary>
    static void OnSceneGUI(SceneView sceneView)
    {
        if (!isGitRepository || string.IsNullOrEmpty(cachedBranchInfo))
            return;
            
        // Create an overlay in the bottom-left corner of the Scene view
        Handles.BeginGUI();
        
        // Calculate position for bottom-left corner
        Vector2 statusSize = new Vector2(300, 20);
        Rect statusRect = new Rect(10, sceneView.position.height - 30, statusSize.x, statusSize.y);
        
        // Draw a subtle background
        Color originalColor = GUI.color;
        GUI.color = new Color(0, 0, 0, 0.3f); // Semi-transparent background
        GUI.Box(statusRect, "", EditorStyles.helpBox);
        GUI.color = originalColor;
        
        // Draw the git information
        GUI.Label(statusRect, cachedBranchInfo, statusBarStyle);
        
        Handles.EndGUI();
    }
    
    /// <summary>
    /// Alternative approach: Create a persistent editor window that looks like a status bar
    /// This window docks at the bottom and shows Git information
    /// </summary>
    public class GitStatusWindow : EditorWindow
    {
        private Vector2 scrollPosition = Vector2.zero; // Track scroll position
        
        [MenuItem("Tools/Show Git Status Bar")]
        public static void ShowWindow()
        {
            var window = GetWindow<GitStatusWindow>("Git Status");
            window.minSize = new Vector2(200, 25);
            window.maxSize = new Vector2(4000, 600); // Increased max height for better scrolling
            window.position = new Rect(0, Screen.currentResolution.height - 100, 400, 25);
        }
        
        void OnGUI()
        {
            if (statusBarStyle == null) InitializeStatusBarStyle();
            
            EditorGUILayout.BeginVertical();
            
            // Main status line
            EditorGUILayout.BeginHorizontal();
            
            if (!string.IsNullOrEmpty(cachedBranchInfo))
            {
                GUILayout.Label(cachedBranchInfo, statusBarStyle);
            }
            else
            {
                GUILayout.Label("Not a Git repository", statusBarStyle);
            }
            
            GUILayout.FlexibleSpace();
            
            // Toggle button for detailed file view
            if (GUILayout.Button(showDetailedFiles ? "▼" : "▶", GUILayout.Width(20), GUILayout.Height(18)))
            {
                showDetailedFiles = !showDetailedFiles;
                // Adjust window size based on detail view
                if (showDetailedFiles)
                {
                    minSize = new Vector2(400, 100);
                    maxSize = new Vector2(4000, 600); // Allow more height for scrolling
                }
                else
                {
                    minSize = new Vector2(200, 25);
                    maxSize = new Vector2(4000, 25);
                }
            }
            
            if (GUILayout.Button("↻", GUILayout.Width(20), GUILayout.Height(18)))
            {
                UpdateGitInformation();
            }
            
            EditorGUILayout.EndHorizontal();
            
            // Detailed file list (if enabled)
            if (showDetailedFiles && changedFiles.Length > 0)
            {
                EditorGUILayout.Space();
                EditorGUILayout.LabelField("Changed Files:", EditorStyles.boldLabel);
                
                // Create a scrollable area for the file list
                scrollPosition = EditorGUILayout.BeginScrollView(
                    scrollPosition, 
                    false, // horizontal scrollbar
                    true,  // vertical scrollbar
                    GUILayout.MinHeight(50),
                    GUILayout.MaxHeight(300) // Limit max height to keep window manageable
                );
                
                EditorGUILayout.BeginVertical(EditorStyles.helpBox);
                
                foreach (string fileInfo in changedFiles)
                {
                    if (string.IsNullOrEmpty(fileInfo)) continue;
                    
                    EditorGUILayout.BeginHorizontal();
                    
                    // Parse the git status format: "XY filename"
                    string statusIndicator = fileInfo.Length >= 2 ? fileInfo.Substring(0, 2) : "??";
                    string fileName = fileInfo.Length > 3 ? fileInfo.Substring(3) : fileInfo;
                    
                    // Color code based on file status
                    Color statusColor = GetFileStatusColor(statusIndicator);
                    string statusText = GetFileStatusText(statusIndicator);
                    
                    // Show status with color
                    GUI.color = statusColor;
                    GUILayout.Label(statusText, GUILayout.Width(70)); // Slightly wider for better readability
                    GUI.color = Color.white;
                    
                    // Show filename (clickable to reveal in project)
                    if (GUILayout.Button(fileName, EditorStyles.linkLabel))
                    {
                        // Try to ping the file in the project window
                        string assetPath = "Assets/" + fileName;
                        Object asset = AssetDatabase.LoadAssetAtPath<Object>(assetPath);
                        if (asset != null)
                        {
                            EditorGUIUtility.PingObject(asset);
                        }
                        else
                        {
                            // If not in Assets, try to open the file location in file explorer
                            string fullPath = Path.Combine(gitRepositoryRoot ?? (Application.dataPath + "/.."), fileName);
                            if (File.Exists(fullPath))
                            {
                                EditorUtility.RevealInFinder(fullPath);
                            }
                        }
                    }
                    
                    EditorGUILayout.EndHorizontal();
                }
                
                EditorGUILayout.EndVertical();
                EditorGUILayout.EndScrollView();
                
                // Show helpful info about the file count
                if (changedFiles.Length > 10)
                {
                    EditorGUILayout.LabelField($"Showing {changedFiles.Length} changed files", EditorStyles.centeredGreyMiniLabel);
                }
            }
            else if (showDetailedFiles && changedFiles.Length == 0)
            {
                EditorGUILayout.Space();
                EditorGUILayout.LabelField("Working directory clean ✓", EditorStyles.centeredGreyMiniLabel);
            }
            
            EditorGUILayout.EndVertical();
        }
        
        void Update()
        {
            // Keep the window updated
            Repaint();
        }
    }
    
    /// <summary>
    /// Main update loop - runs periodically to refresh Git information
    /// </summary>
    static void UpdateGitInformation()
    {
        if (EditorApplication.timeSinceStartup - lastUpdateTime < updateInterval)
            return;
            
        lastUpdateTime = EditorApplication.timeSinceStartup;
        
        try
        {
            if (!IsGitRepository())
            {
                isGitRepository = false;
                cachedBranchInfo = "";
                return;
            }
            
            isGitRepository = true;
            
            string branchName = ExecuteGitCommand("rev-parse --abbrev-ref HEAD").Trim();
            string statusOutput = ExecuteGitCommand("status --porcelain");
            
            // Parse and store individual file changes
            ParseChangedFiles(statusOutput);
            
            string statusSummary = ParseStatusForStatusBar(statusOutput);
            
            // Create a nicely formatted status string with icons
            if (string.IsNullOrEmpty(statusSummary))
            {
                cachedBranchInfo = $"🌿 {branchName} ✓ Clean";
            }
            else
            {
                cachedBranchInfo = $"🌿 {branchName} • {statusSummary}";
            }
            
            // Console logging removed - only manual refresh will log
        }
        catch (System.Exception e)
        {
            cachedBranchInfo = $"⚠️ Git Error: {e.Message.Split('\n')[0]}";
            isGitRepository = false;
            changedFiles = new string[0]; // Clear file list on error
        }
    }
    
    static bool IsGitRepository()
    {
        string currentPath = Path.GetFullPath(Application.dataPath + "/..");
        
        // Walk up the directory tree looking for .git folder (same behavior as Git)
        while (!string.IsNullOrEmpty(currentPath))
        {
            string gitPath = Path.Combine(currentPath, ".git");
            
            if (Directory.Exists(gitPath) || File.Exists(gitPath))
            {
                // Store the git repository root for use in Git commands
                gitRepositoryRoot = currentPath;
                return true;
            }
            
            // Move up one directory level
            DirectoryInfo parent = Directory.GetParent(currentPath);
            if (parent == null)
                break;
                
            currentPath = parent.FullName;
        }
        
        gitRepositoryRoot = null;
        return false;
    }
    
    static string ExecuteGitCommand(string arguments)
    {
        // Use the discovered Git repository root, or fall back to Unity project root
        string workingDirectory = gitRepositoryRoot ?? (Application.dataPath + "/..");
        
        ProcessStartInfo startInfo = new ProcessStartInfo()
        {
            FileName = "git",
            Arguments = arguments,
            UseShellExecute = false,
            RedirectStandardOutput = true,
            RedirectStandardError = true,
            CreateNoWindow = true,
            WorkingDirectory = workingDirectory
        };
        
        using (Process process = Process.Start(startInfo))
        {
            string output = process.StandardOutput.ReadToEnd();
            string error = process.StandardError.ReadToEnd();
            
            process.WaitForExit();
            
            if (process.ExitCode != 0)
            {
                throw new System.Exception($"Git command failed: {error}");
            }
            
            return output;
        }
    }
    
    static string ParseStatusForStatusBar(string statusOutput)
    {
        if (string.IsNullOrEmpty(statusOutput.Trim()))
            return "";
        
        string[] lines = statusOutput.Split('\n');
        int totalChanges = 0;
        int staged = 0;
        int untracked = 0;
        
        foreach (string line in lines)
        {
            if (string.IsNullOrEmpty(line)) continue;
            
            totalChanges++;
            
            char stagedStatus = line[0];
            if (stagedStatus != ' ' && stagedStatus != '?') staged++;
            if (stagedStatus == '?') untracked++;
        }
        
        if (totalChanges == 0) return "";
        
        string result = "";
        if (staged > 0) result += $"{staged} staged";
        if (untracked > 0)
        {
            if (!string.IsNullOrEmpty(result)) result += ", ";
            result += $"{untracked} untracked";
        }
        
        int unstaged = totalChanges - staged - untracked;
        if (unstaged > 0)
        {
            if (!string.IsNullOrEmpty(result)) result += ", ";
            result += $"{unstaged} modified";
        }
        
        return string.IsNullOrEmpty(result) ? $"{totalChanges} changes" : result;
    }
    
    /// <summary>
    /// Parse the git status output and store individual file changes
    /// </summary>
    static void ParseChangedFiles(string statusOutput)
    {
        if (string.IsNullOrEmpty(statusOutput.Trim()))
        {
            changedFiles = new string[0];
            return;
        }
        
        // Split into lines and filter out empty ones
        string[] lines = statusOutput.Split('\n');
        System.Collections.Generic.List<string> fileList = new System.Collections.Generic.List<string>();
        
        foreach (string line in lines)
        {
            if (!string.IsNullOrEmpty(line.Trim()))
            {
                fileList.Add(line);
            }
        }
        
        changedFiles = fileList.ToArray();
    }
    
    /// <summary>
    /// Get appropriate color for different file status types
    /// </summary>
    static Color GetFileStatusColor(string statusIndicator)
    {
        if (statusIndicator.Length < 2) return Color.white;
        
        char staged = statusIndicator[0];
        char working = statusIndicator[1];
        
        // Priority: staged changes (first character)
        switch (staged)
        {
            case 'A': return Color.green;      // Added/new file
            case 'M': return Color.yellow;     // Modified
            case 'D': return Color.red;        // Deleted
            case 'R': return Color.cyan;       // Renamed
            case 'C': return Color.magenta;    // Copied
            case '?': return Color.grey;       // Untracked
        }
        
        // If not staged, check working tree changes
        switch (working)
        {
            case 'M': return Color.yellow;     // Modified in working tree
            case 'D': return Color.red;        // Deleted in working tree
            default: return Color.white;
        }
    }
    
    /// <summary>
    /// Get human-readable text for file status indicators
    /// </summary>
    static string GetFileStatusText(string statusIndicator)
    {
        if (statusIndicator.Length < 2) return "??";
        
        char staged = statusIndicator[0];
        char working = statusIndicator[1];
        
        // Handle staged changes first
        switch (staged)
        {
            case 'A': return "Added";
            case 'M': return working == 'M' ? "Modified*" : "Staged";  // * means also modified in working tree
            case 'D': return "Deleted";
            case 'R': return "Renamed";
            case 'C': return "Copied";
            case '?': return "Untracked";
        }
        
        // Handle working tree only changes
        switch (working)
        {
            case 'M': return "Modified";
            case 'D': return "Deleted";
            default: return statusIndicator.Trim();
        }
    }
    
    /// <summary>
    /// Console-based status display as an alternative
    /// This prints Git status to Unity's Console window
    /// </summary>
    [MenuItem("Tools/Print Git Status to Console")]
    static void PrintGitStatusToConsole()
    {
        UpdateGitInformation();
        if (!string.IsNullOrEmpty(cachedBranchInfo))
        {
            UnityEngine.Debug.Log($"<color=green>Git Status:</color> {cachedBranchInfo}");
        }
        else
        {
            UnityEngine.Debug.Log("<color=yellow>Not in a Git repository or Git not available</color>");
        }
    }
    
    /// <summary>
    /// Manual refresh option accessible from Unity's menu
    /// </summary>
    [MenuItem("Tools/Refresh Git Status")]
    static void ManualRefresh()
    {
        lastUpdateTime = 0; // Force immediate update
        UpdateGitInformation();
    }
    
    /// <summary>
    /// Quick access to open Git status window
    /// </summary>
    [MenuItem("Tools/Git Status Display Options/Scene View Overlay", false, 1)]
    static void ToggleSceneViewDisplay()
    {
        bool currentState = EditorPrefs.GetBool("GitStatusBar.ShowInSceneView", true);
        EditorPrefs.SetBool("GitStatusBar.ShowInSceneView", !currentState);
        
        string status = !currentState ? "enabled" : "disabled";
        UnityEngine.Debug.Log($"Git status scene view overlay {status}");
    }
}