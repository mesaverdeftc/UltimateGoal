package org.firstinspires.ftc.teamcode;

//import org.firstinspires.ftc.teamcode.Dependencies.Constants;
//import org.firstinspires.ftc.teamcode.Dependencies.Constants.Stack;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class EasyOpenCVDetectorPipeline extends OpenCvPipeline
{


//    Some color constants

    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar GREEN = new Scalar(0, 255, 0);


//    The core values which define the location and size of the sample regions


    public static int TOP_POINT=63;//b65
    public static int LEFT_POINT=280;//b95
    public static int REGION_WIDTH = 40;//b40
    public static int REGION_HEIGHT = 40;//b40
    public static int QUAD = 149;
    public static int SINGLE = 135;


    Point region1_pointA = new Point(
            LEFT_POINT,
            TOP_POINT);
    Point region1_pointB = new Point(
            LEFT_POINT + REGION_WIDTH,
            TOP_POINT + REGION_HEIGHT);


//    Working variables

    Mat region1_Cb;
    Mat YCrCb = new Mat();
    Mat Cb = new Mat();
    int avg1;

    // Volatile since accessed by OpMode thread w/o synchronization
    public volatile Constants.Stack position = Constants.Stack.NONE;


    @Override
    public void init(Mat firstFrame)
    {
        Imgproc.cvtColor(firstFrame, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 1);

        region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
    }

    @Override
    public Mat processFrame(Mat input)
    {

        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 1);

        avg1 = (int) Core.mean(region1_Cb).val[0];

        Imgproc.rectangle(
                input, // Buffer to draw on
                region1_pointA, // First point which defines the rectangle
                region1_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                1); // Thickness of the rectangle lines

        if(avg1 > QUAD)position = Constants.Stack.QUAD;
        else if (avg1 > SINGLE)position = Constants.Stack.SINGLE;
        else position = Constants.Stack.NONE;


        return input;
    }

    public void resetPoints(boolean isBlueSide){
        TOP_POINT=isBlueSide?65:63;
        LEFT_POINT=isBlueSide?95:280;
        region1_pointA = new Point(
                LEFT_POINT,
                TOP_POINT);
        region1_pointB = new Point(
                LEFT_POINT + REGION_WIDTH,
                TOP_POINT + REGION_HEIGHT);
    }


    public Constants.Stack getPosition(){
        return position;
    }

    public int getAnalysis()
    {
        return avg1;
    }
}