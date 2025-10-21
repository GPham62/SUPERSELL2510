using System;
using OpenCvSharp;
using OpenCvSharp.Demo;
using UnityEngine;

public class HeadTracker : WebCamera
{
    public TextAsset faces;
    public TextAsset eyes;
    public TextAsset shapes;
    [SerializeField] private double Threshold = 2.0;
    [SerializeField] private int SampleCount = 2;
    [SerializeField] private int Downscale = 256;
    [SerializeField] private int SkipRate = 0;
    [SerializeField] private Transform head;
    private FaceProcessorLive<WebCamTexture> processor;
    protected override void Awake()
    {
        base.Awake();
        forceFrontalCamera = true;

        processor = new FaceProcessorLive<WebCamTexture>();
        processor.Initialize(faces.text, eyes.text, shapes.bytes);
        processor.DataStabilizer.Enabled = true;
        processor.DataStabilizer.Threshold = Threshold;
        processor.DataStabilizer.SamplesCount = SampleCount;

        processor.Performance.Downscale = Downscale;
        processor.Performance.SkipRate = SkipRate;
    }

    protected override bool ProcessTexture(WebCamTexture input, ref Texture2D output)
    {
        processor.ProcessTexture(input, TextureParameters);
        
        processor.MarkDetected();

        output = OpenCvSharp.Unity.MatToTexture(processor.Image, output);

        if (processor.Faces != null && processor.Faces.Count > 0)
        {
            var face = processor.Faces[0];
            Point2f[] points2D = new Point2f[]
            {
                face.Marks[30],
                face.Marks[8],
                face.Marks[36],
                face.Marks[45],
                face.Marks[48],
                face.Marks[54]
            };
            var (yaw, pitch, roll) = EstimateHeadPose(processor.Image, points2D);
            if (head != null)
            {
                head.rotation = Quaternion.Euler((float)pitch, (float)-yaw, (float)-roll);
            }
        }
        return true;
    }

    private (double yaw, double pitch, double roll) EstimateHeadPose(Mat frame, Point2f[] marks)
    {
        // 3D model reference points
        var modelPoints = new[]
        {
            new Point3f(0.0f, 0.0f, 0.0f),             // Nose tip
            new Point3f(0.0f, -330.0f, -65.0f),        // Chin
            new Point3f(-225.0f, 170.0f, -135.0f),     // Left eye left corner
            new Point3f(225.0f, 170.0f, -135.0f),      // Right eye right corner
            new Point3f(-150.0f, -150.0f, -125.0f),    // Left mouth corner
            new Point3f(150.0f, -150.0f, -125.0f)      // Right mouth corner
        };

        // Camera intrinsics
        double focalLength = frame.Cols;
        Point2d center = new Point2d(frame.Cols / 2, frame.Rows / 2);

        Mat cameraMatrix = new Mat(3, 3, MatType.CV_64FC1);
        cameraMatrix.Set(0, 0, focalLength);
        cameraMatrix.Set(0, 1, 0);
        cameraMatrix.Set(0, 2, center.X);
        cameraMatrix.Set(1, 0, 0);
        cameraMatrix.Set(1, 1, focalLength);
        cameraMatrix.Set(1, 2, center.Y);
        cameraMatrix.Set(2, 0, 0);
        cameraMatrix.Set(2, 1, 0);
        cameraMatrix.Set(2, 2, 1);

        // No lens distortion
        Mat distCoeffs = Mat.Zeros(4, 1, MatType.CV_64FC1);

        // Compute rotation + translation
        Mat rvec = new Mat();
        Mat tvec = new Mat();
        Cv2.SolvePnP(InputArray.Create(modelPoints), InputArray.Create(marks), cameraMatrix, distCoeffs, rvec, tvec, false, SolvePnPFlags.Iterative);
        // Convert rotation vector to rotation matrix
        Mat rotM = new Mat();
        Cv2.Rodrigues(rvec, rotM);

        // Convert rotation matrix to Euler angles
        double[] angles = RotationMatrixToEulerAngles(rotM);

        // Return yaw, pitch, roll
        return (angles[1], angles[0], angles[2]);
    }

    
    private static double[] RotationMatrixToEulerAngles(Mat R)
    {
        double sy = Math.Sqrt(R.At<double>(0, 0) * R.At<double>(0, 0) + R.At<double>(1, 0) * R.At<double>(1, 0));
        bool singular = sy < 1e-6;
        double x, y, z;
        if (!singular)
        {
            x = Math.Atan2(R.At<double>(2, 1), R.At<double>(2, 2));
            y = Math.Atan2(-R.At<double>(2, 0), sy);
            z = Math.Atan2(R.At<double>(1, 0), R.At<double>(0, 0));
        }
        else
        {
            x = Math.Atan2(-R.At<double>(1, 2), R.At<double>(1, 1));
            y = Math.Atan2(-R.At<double>(2, 0), sy);
            z = 0;
        }

        return new double[] { x * Mathf.Rad2Deg, y * Mathf.Rad2Deg, z * Mathf.Rad2Deg };
    }
}