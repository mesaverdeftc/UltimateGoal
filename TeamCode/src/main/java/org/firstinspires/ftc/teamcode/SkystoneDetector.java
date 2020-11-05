package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SkystoneDetector extends OpenCvPipeline {
    private Mat workingMatrix = new Mat();
    public String position = "LEFT";
    public double analysis = 0;

    private Mat matLeft, matCenter, matRight;

//    @Override
//    public void init (Mat firstFrame) {
//        Imgproc.cvtColor(firstFrame, workingMatrix, Imgproc.COLOR_RGB2YCrCb);
//
//        matLeft = workingMatrix.submat(120, 150, 10, 50);
//        matCenter = workingMatrix.submat(120, 150, 80, 120);
//        matRight = workingMatrix.submat(120, 150, 150, 190);
//    }

    @Override
    public final Mat processFrame(Mat input) {

        input.copyTo(workingMatrix);

        if (workingMatrix.empty()) {
            return input;
        }

        Imgproc.cvtColor(workingMatrix, workingMatrix, Imgproc.COLOR_RGB2YCrCb);

        matLeft = workingMatrix.submat(120, 150, 10, 50);
        matCenter = workingMatrix.submat(120, 150, 80, 120);
        matRight = workingMatrix.submat(120, 150, 150, 190);

        Imgproc.rectangle(workingMatrix, new Rect(10, 120, 40, 30), new Scalar(0, 255,0));
        Imgproc.rectangle(workingMatrix, new Rect(80, 120, 40, 30), new Scalar(0, 255,0));
        Imgproc.rectangle(workingMatrix, new Rect(150, 120, 40, 30), new Scalar(0, 255,0));


        double leftTotal = Core.sumElems(matLeft).val[1];
        double centerTotal = Core.sumElems(matCenter).val[1];
        double rightTotal = Core.sumElems(matRight).val[1];

        if (leftTotal > centerTotal) {
            if (leftTotal > rightTotal) {
                // left is skystone
                position = "LEFT";
                analysis = leftTotal;
            } else {
                // right is skystone
                position = "RIGHT";
                analysis = rightTotal;
            }
        } else {
            if (centerTotal > rightTotal) {
                // center is skystone
                position = "CENTER";
                analysis = centerTotal;
            } else {
                // right is skystone
                position = "RIGHT";
                analysis = rightTotal;
            }
        }

        return workingMatrix;
    }
}
