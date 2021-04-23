package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;

import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;


@TeleOp (name = "GoalDetection OpMode")
public class GoalDetectionTestOpMode extends LinearOpMode {

    private OpenCvCamera camera;
    private UGAngleHighGoalPipeline pipeline;
    private Mat mat;
    public UGAngleHighGoalPipeline.Fraction fraction;

    //Pinhole Camera Variables
    protected double centerX;
    protected double centerY;
    private double cameraPitchOffset = 0;
    private double cameraYawOffset = 0;

    private double fov = 90;
    private double horizontalFocalLength;
    private double verticalFocalLength;

    private int imageWidth = 320; // width  of wanted camera resolution
    private int imageHeight = 240; // height of wanted camera resolution

    //Aspect ratio (3 by 2 by default)
    public static int horizontalRatio;
    public static int verticalRatio;

    //Boundary Line (Only detects above this to eliminate field tape)
    public static int BOUNDARY = 130;

    //Other Variables
    public List<MatOfPoint> blueContours;
    public MatOfPoint biggestBlueContour;
    public Rect blueRect;
    public int minThreshold, maxThreshold;
    private Mat blueThreshold;
    private Mat blueChannel;

    public Mat matYCrCb;
    public Mat CbFrame;
    public Mat MaskFrame;
    public Mat ContourFrame;

    public static double CENTER_X_OFFSET = 0;
    public static double CENTER_Y_OFFSET = 0;

    Point upperLeftCorner;
    Point upperRightCorner;
    Point lowerLeftCorner;
    Point lowerRightCorner;
    Point upperMiddle;
    Point lowerMiddle;
    Point leftMiddle;
    Point rightMiddle;

    static class Fraction {
        private int numerator, denominator;

        Fraction(long a, long b) {
            numerator = (int) (a / gcd(a, b));
            denominator = (int) (b / gcd(a, b));
        }

        /**
         * @return the greatest common denominator
         */
        private long gcd(long a, long b) {
            return b == 0 ? a : gcd(b, a % b);
        }

        public int getNumerator() {
            return numerator;
        }

        public int getDenominator() {
            return denominator;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        matYCrCb = new Mat();
        CbFrame = new Mat();
        MaskFrame = new Mat();
        ContourFrame = new Mat();

        biggestBlueContour = new MatOfPoint();
        blueContours = new ArrayList<MatOfPoint>();
        blueRect = new Rect();
        blueThreshold = new Mat();
        minThreshold = 155;
        maxThreshold = 200;
        blueChannel = new Mat();

        upperLeftCorner = new Point();
        upperRightCorner = new Point();
        lowerLeftCorner = new Point();
        lowerRightCorner = new Point();

        upperMiddle = new Point();
        lowerMiddle = new Point();
        leftMiddle = new Point();
        rightMiddle = new Point();

        //Init Camera
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory
                .getInstance()
                .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        pipeline = new UGAngleHighGoalPipeline(0, 0, 0);
        camera.openCameraDeviceAsync(() -> camera.startStreaming(imageWidth, imageHeight, OpenCvCameraRotation.UPRIGHT));
        camera.setPipeline(pipeline);

        telemetry.addData("this works", centerX);
        telemetry.update();
        idle();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("this also works", 12);
            telemetry.update();

            idle();
        }
    }
}