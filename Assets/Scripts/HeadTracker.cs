using System;
using OpenCvSharp;
using OpenCvSharp.Demo;
using UnityEngine;

public class HeadTracker : WebCamera
{
    private Mat image;
    protected override bool ProcessTexture(WebCamTexture input, ref Texture2D output)
    {
        image = OpenCvSharp.Unity.TextureToMat(input);
        
        //processing stuff
        if (output == null)
        {
            output = OpenCvSharp.Unity.MatToTexture(image);
        }
        else
        {
            OpenCvSharp.Unity.MatToTexture(image, output);
        }

        return true;
    }
}