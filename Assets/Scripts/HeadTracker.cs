using System;
using OpenCvSharp;
using OpenCvSharp.Demo;
using UnityEngine;

public class HeadTracker : WebCamera
{
    
    // [SerializeField] private FlipMode ImageFlip;
    // [SerializeField] private float Threshold = 96.4f;
    // [SerializeField] private bool ShowProcessingImage = true;
    // [SerializeField] private float CurveAccuracy = 10f;
    // [SerializeField] private float MinArea = 5000f;
    //
    // private Mat _image;
    // private Mat _processImage = new Mat();
    // private Point[][] _contours;
    // private HierarchyIndex[] _hierarchy;
    // protected override bool ProcessTexture(WebCamTexture input, ref Texture2D output)
    // {
    //     _image = OpenCvSharp.Unity.TextureToMat(input);
    //     
    //     Cv2.Flip(_image, _image, ImageFlip);
    //     Cv2.CvtColor(_image, _processImage, ColorConversionCodes.BGR2GRAY);
    //     Cv2.Threshold(_processImage, _processImage, Threshold, 255, ThresholdTypes.BinaryInv);
    //     Cv2.FindContours(_processImage, out _contours, out _hierarchy, RetrievalModes.Tree, ContourApproximationModes.ApproxSimple);
    //     foreach (Point[] contour in _contours)
    //     {
    //         Point[] points = Cv2.ApproxPolyDP(contour, CurveAccuracy, true);
    //         var area = Cv2.ContourArea(contour);
    //
    //         if (area > MinArea)
    //         {
    //             DrawContour(_processImage, new Scalar(127, 127, 127), 2, points);
    //         }
    //     }
    //     if (output == null)
    //     {
    //         output = OpenCvSharp.Unity.MatToTexture(ShowProcessingImage? _processImage : _image);
    //     }
    //     else
    //     {
    //         OpenCvSharp.Unity.MatToTexture(ShowProcessingImage ? _processImage : _image, output);
    //     }
    //
    //     return true;
    // }
    //
    // private void DrawContour(Mat Image, Scalar Color, int Thickness, Point[] Points)
    // {
    //     for (int i = 1; i < Points.Length; i++)
    //     {
    //         Cv2.Line(Image, Points[i - 1], Points[i], Color, Thickness);
    //     }
    //     Cv2.Line(Image, Points[Points.Length - 1], Points[0], Color, Thickness);
    // }

    public TextAsset faces;
    public TextAsset eyes;
    public TextAsset shapes;
    
    protected override void Awake()
    {
        base.Awake();
        
    }

    protected override bool ProcessTexture(WebCamTexture input, ref Texture2D output)
    {
        return true;
    }
}