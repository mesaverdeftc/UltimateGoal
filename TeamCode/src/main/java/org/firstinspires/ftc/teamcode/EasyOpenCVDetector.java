package org.firstinspires.ftc.teamcode;

import android.util.Log;

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.Dependencies.Constants;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

//import static org.firstinspires.ftc.teamcode.Dependencies.Constants.Stack.NONE;


public class EasyOpenCVDetector {
    OpenCvInternalCamera phoneCam;
    EasyOpenCVDetectorPipeline pipeline;
    public Constants.Stack result;
    protected Constants.Stack lastResult;
    public int noneSeen=0;

    public EasyOpenCVDetector(HardwareMap hardwareMap) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);


    }

    public void init(boolean isBlueSide){
        //FtcDashboard dashboard = FtcDashboard.getInstance();
        //Telemetry dashboardTelemetry = dashboard.getTelemetry();
        //TelemetryPacket packet = new TelemetryPacket();
        //FtcDashboard.getInstance().startCameraStream(phoneCam, 30);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        pipeline = new EasyOpenCVDetectorPipeline();
        pipeline.resetPoints(isBlueSide);
        phoneCam.setPipeline(pipeline);
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });

    }

    public void KILLIT(){
        phoneCam.stopStreaming();
    }



    public void update(Telemetry telemetry){
        result=pipeline.getPosition();
        telemetry.addData("Stack: ", result);
        telemetry.addData("Average: ", pipeline.getAnalysis());
    }

    public void fixResult(){
        //if (lastResult == null && result == null) {
        //    result = NONE;  // when detection fails, default to NONE
        //    Log.d("UG-BaseAuto", "Camera did not initialize, defaulted to NONE.");
        //}else {
        //    lastResult = result != NONE ? result : lastResult;
        //    if (lastResult != NONE) result = lastResult;
        //    Log.d("UG-BaseAuto", "TOFDResult:" + result);
        //}
    }
}