package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.*;
import java.util.stream.Collectors;

import static java.util.Optional.of;


public class UGBasicHighGoalPipeline extends OpenCvPipeline {

    protected double centerX;
    protected double centerY;

    public int minThreshold, maxThreshold;
    private Mat blueThreshold;
    private Mat redThreshold;

    private Mat matYCrCb;
    private Mat redChannel;
    private Mat blueChannel;

    public static int BOUNDARY = 130;
    public static double CENTER_X_OFFSET = 0;
    public static double CENTER_Y_OFFSET = 0;
    private int imageWidth = 320; // width  of wanted camera resolution
    private int imageHeight = 240; // height of wanted camera resolution

    Point upperLeftCorner;
    Point upperRightCorner;
    Point lowerLeftCorner;
    Point lowerRightCorner;
    Point upperMiddle;
    Point lowerMiddle;
    Point leftMiddle;
    Point rightMiddle;

    private List<MatOfPoint> redContours;
    private List<MatOfPoint> blueContours;
    private MatOfPoint biggestBlueContour;
    private MatOfPoint biggestRedContour;
    private Rect blueRect, redRect;

    private Rect bluePowershot, bluePowershot2, bluePowershot3, redPowershot;

    public UGBasicHighGoalPipeline() {

        matYCrCb = new Mat();
        redChannel = new Mat();
        blueChannel = new Mat();

        blueThreshold = new Mat();
        redThreshold = new Mat();

        blueContours = new ArrayList<MatOfPoint>();
        redContours = new ArrayList<MatOfPoint>();

        biggestBlueContour = new MatOfPoint();
        biggestRedContour = new MatOfPoint();

        blueRect = new Rect();
        redRect = new Rect();

        minThreshold = 155;
        maxThreshold = 200;

    }

    @Override
    public void init(Mat mat) {
        super.init(mat);
        int imageWidth = mat.width();
        int imageHeight = mat.height();

        int imageArea = imageWidth * imageHeight;
        centerX = ((double) imageWidth / 2) - 0.5;
        centerY = ((double) imageHeight / 2) - 0.5;

        upperLeftCorner = new Point();
        upperRightCorner = new Point();
        lowerLeftCorner = new Point();
        lowerRightCorner = new Point();

        upperMiddle = new Point();
        lowerMiddle = new Point();
        leftMiddle = new Point();
        rightMiddle = new Point();

        centerX = ((double) imageWidth / 2) - 0.5;
        centerY = ((double) imageHeight / 2) - 0.5;
    }

    public boolean filterContours(MatOfPoint contour) {
        return Imgproc.contourArea(contour) > 30;
    }
    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, matYCrCb, Imgproc.COLOR_RGB2YCrCb);

        Core.extractChannel(matYCrCb, redChannel, 1);
        Core.extractChannel(matYCrCb, blueChannel, 2);

        // Blue threshold
        Imgproc.threshold(blueChannel, blueThreshold, minThreshold, maxThreshold, Imgproc.THRESH_BINARY);
        // Red threshold
        Imgproc.threshold(redChannel, redThreshold, minThreshold, maxThreshold, Imgproc.THRESH_BINARY);

        blueContours.clear();
        redContours.clear();

        Imgproc.findContours(blueThreshold, blueContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(redThreshold, redContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        blueContours = blueContours.stream().filter(i -> filterContours(i) && (((double) Imgproc.boundingRect(i).height / Imgproc.boundingRect(i).width) > 1) &&(((double) Imgproc.boundingRect(i).height / Imgproc.boundingRect(i).width) < 2)).collect(Collectors.toList());
        redContours = redContours.stream().filter(i -> filterContours(i) && (((double) Imgproc.boundingRect(i).width / Imgproc.boundingRect(i).height) > 1) &&(((double) Imgproc.boundingRect(i).width / Imgproc.boundingRect(i).height) < 2)).collect(Collectors.toList());

        Imgproc.drawContours(input, blueContours, -1, new Scalar(255, 255, 0));

        if (blueContours.size() != 0) {
            biggestBlueContour = Collections.max(blueContours, new Comparator<MatOfPoint>() {
                @Override
                public int compare(MatOfPoint t0, MatOfPoint t1) {
                    return Double.compare(Imgproc.boundingRect(t0).width, Imgproc.boundingRect(t1).width);
                }
            });
            // TODO switch width and height bc the camera is sideways on robot

            blueRect = Imgproc.boundingRect(biggestBlueContour);


            int x = blueRect.x - blueRect.width/3;
            int y = blueRect.y + blueRect.height + (blueRect.height / 10);
            Imgproc.circle(input, new Point(x,y), 2, new Scalar(255, 0, 0), 4);


            int width = (blueRect.height/8);
            int height = blueRect.width/3;

            bluePowershot = new Rect(x, y, height, width);
            bluePowershot2 = new Rect(x + blueRect.width/10, (y + blueRect.width/2)-2, height, width);
            bluePowershot3 = new Rect(x + blueRect.width/6, (y + blueRect.width)-6, height, width);

            Imgproc.rectangle(input, blueRect, new Scalar(0, 140, 255), 3);
            Imgproc.rectangle(input, bluePowershot, new Scalar(110, 220, 225), 3);
            Imgproc.rectangle(input, bluePowershot2, new Scalar(110, 220, 225), 3);
            Imgproc.rectangle(input, bluePowershot3, new Scalar(110, 220, 225), 3);

            System.out.println((double) blueRect.width / (double) blueRect.height);

        } else {
            blueRect = null;
        }
//        if (redContours.size() != 0) {
//            biggestRedContour = Collections.max(redContours, new Comparator<MatOfPoint>() {
//                @Override
//                public int compare(MatOfPoint t0, MatOfPoint t1) {
//                    return Double.compare(Imgproc.boundingRect(t0).width, Imgproc.boundingRect(t1).width);
//                }
//            });
//
//            redRect = Imgproc.boundingRect(biggestRedContour);
//
//            int x = redRect.x - (redRect.width / 5) * 4;
//            int y = redRect.y + redRect.height;
//
//            int width = (redRect.width/5) * 4;
//            int height = redRect.height/3;
//
//            redPowershot = new Rect(x, y, width, height);
//
//            Imgproc.rectangle(input, redRect, new Scalar(255, 0, 0), 3);
//            Imgproc.rectangle(input, redPowershot, new Scalar(222, 23, 56), 3);
//
//            System.out.println((double) redRect.width / (double) redRect.height);
//
//        } else {
//            redRect = null;
//        }
//
//        Imgproc.line(input, new Point(centerX, centerY + 5), new Point(centerX, centerY - 5), new Scalar(0, 0, 0));  //crosshair vertical
//        Imgproc.line(input, new Point(centerX + 5, centerY), new Point(centerX - 5, centerY), new Scalar(0, 0, 0));  //crosshair horizontal
//
//        //draw center offset (where you want the goal to be)
//        Imgproc.line(input, new Point(centerX + CENTER_X_OFFSET, centerY + CENTER_Y_OFFSET + 5), new Point(centerX + CENTER_X_OFFSET, centerY + CENTER_Y_OFFSET - 5), new Scalar(255, 0, 0));  //crosshair vertical
//        Imgproc.line(input, new Point(centerX + CENTER_X_OFFSET + 5, centerY + CENTER_Y_OFFSET), new Point(centerX + CENTER_X_OFFSET - 5, centerY + CENTER_Y_OFFSET), new Scalar(255, 0, 0));  //crosshair horizontal
//
//        //draw boundary line
//        Imgproc.line(input, new Point(0, BOUNDARY), new Point(this.imageWidth, BOUNDARY), new Scalar(0, 0, 255));
//
//        //draw contours
//        Imgproc.drawContours(input, blueContours, -1, new Scalar(255, 255, 0));
//
//        //draw bounding box
//        Imgproc.rectangle(input, blueRect, new Scalar(0, 255, 0), 1);

        //Draw Corners and Middle
        findCorners();
        if (!biggestBlueContour.empty()) {
            Imgproc.circle(input, upperLeftCorner, 2, new Scalar(0, 255, 0), 4);
            Imgproc.circle(input, upperRightCorner, 2, new Scalar(0, 255, 0), 4);
            Imgproc.circle(input, lowerLeftCorner, 2, new Scalar(0, 255, 0), 4);
            Imgproc.circle(input, lowerRightCorner, 2, new Scalar(0, 255, 0), 4);

            Imgproc.circle(input, upperMiddle, 2, new Scalar(255, 0, 0), 2);
            Imgproc.circle(input, lowerMiddle, 2, new Scalar(255, 0, 0), 2);

            //draw center line
            Imgproc.line(input, lowerMiddle, upperMiddle, new Scalar(255, 0, 0));
        }

        return input;
    }

    public void findCorners(){
        if (biggestBlueContour.empty()) {
            lowerLeftCorner = new Point();
            upperLeftCorner = new Point();
            upperRightCorner = new Point();
            lowerRightCorner = new Point();
        } else {
            RotatedRect blueRect = Imgproc.minAreaRect(new MatOfPoint2f(biggestBlueContour.toArray()));
            Point[] corners = new Point[4];
            blueRect.points(corners);
            lowerLeftCorner = corners[0];
            upperLeftCorner = corners[1];
            upperRightCorner = corners[2];
            lowerRightCorner = corners[3];
        }
    }

    public Rect getRedRect() {
        return redRect;
    }

    public Rect getBlueRect() {
        return blueRect;
    }

    public Rect getRedPowershot() {
        return redPowershot;
    }

    public Rect getBluePowershot() {
        return bluePowershot;
    }

    public boolean isRedVisible() {
        return (redRect != null);
    }

    public boolean isBlueVisible() {
        return (blueRect != null);
    }

    public boolean isRedPowershotVisible() {
        return (redPowershot != null);
    }

    public boolean isBluePowershotVisible() {
        return (bluePowershot != null);
    }

    public Point getCenterofRect(Rect rect) {
        if (rect == null) {
            return new Point(centerX, centerY);
        }
        return new Point(rect.x + rect.width / 2, rect.y + rect.height / 2);
    }


}