using System.Collections.Generic;
using UnityEngine;

public class ROSTransformPublisherBenchmark : MonoBehaviour
{
    [Tooltip("Number of samples to use for averaging.")]
    public int sampleSize = 100;

    Queue<double> timestampQueue = new Queue<double>();
    double lastTime = 0;

    void Start()
    {
        lastTime = Time.realtimeSinceStartupAsDouble;
    }

    /// <summary>
    /// Call this from inside your PublishMessage() function
    /// </summary>
    public void RecordPublish()
    {
        double now = Time.realtimeSinceStartupAsDouble;
        double delta = now - lastTime;

        if (timestampQueue.Count >= sampleSize)
            timestampQueue.Dequeue();

        timestampQueue.Enqueue(delta);
        lastTime = now;

        if (timestampQueue.Count >= 2)
        {
            double avgDelta = 0;
            foreach (var d in timestampQueue)
                avgDelta += d;
            avgDelta /= timestampQueue.Count;

            double hz = 1.0 / avgDelta;
            Debug.Log($"[TF Publish Rate]: {hz:F2} Hz over {timestampQueue.Count} samples");
        }
    }
}
