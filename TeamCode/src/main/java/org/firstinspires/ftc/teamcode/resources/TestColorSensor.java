package org.firstinspires.ftc.teamcode.resources;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.resources.ColorDetector;
import org.firstinspires.ftc.teamcode.resources.HardwarePushbotCam;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Test Color Sensor", group="Autonomous")
@Disabled
public class TestColorSensor extends OpMode {

    private HardwarePushbotCam robot = new HardwarePushbotCam();
    OpenCvCamera camera;
    private ColorDetector detect;


    private int quadrant;

    public static final String VUFORIA_LICENSE_KEY = "ARU7jLz/////AAABmVJhts3aqENQlHXD3Sphd9cBUkA39t41u/ZX/zXPD0MbCALSyHLr5x+QXwAW4anR/m0X4kQbSA3PgA+WQkemtCLr3jClaiY+3a+SmbqQuoTnnFVHh6Rxj35H2k2BjE4qVUWxRTijv/xSSRAmSvg/DYdl4bLv0PeNh4T3ccO4VmGs+T5K5POk88BqD1UttSUFh3srzl5+rb6ee346TZXuYUgs78OaYBnL1iqA5iXim+v0VRAa7htpARl7DMoLu2jnf3k/wG6/bF5IQTKtQQiePhosacImlt2tKno61dcx2znT/Tn5ENCeSvuOfZOYIbX0s/4RgLAOOX0CMpSS4shBoJhgTuNSTEUkbf8je4b+xtxB";

    @Override
    public void init() {
        robot.init(hardwareMap);

//        VuforiaLocalizer.Parameters vuforiaParams = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
//        vuforiaParams.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
//        vuforiaParams.cameraDirection = BuiltinCameraDirection.BACK;
//        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(vuforiaParams);
//        FtcDashboard.getInstance().startCameraStream(vuforia, 0);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        detect = new ColorDetector();
        camera.setPipeline(detect);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                System.out.println(errorCode);
            }
        });

        FtcDashboard.getInstance().startCameraStream(camera, 0);
        telemetry.setMsTransmissionInterval(50);
    }

    public void init_loop() {
        quadrant = detect.getQuadrant();
        telemetry.addData("Quadrant", quadrant);
//        if (detect.avgPosition == null) {
//            telemetry.addLine("Man we really fucked rn");
//        } else {
//            telemetry.addData("Average position", detect.avgPosition.toString());
//        }

        telemetry.update();
    }

    public void start() {

    }

    public void loop() {

    }

    public void stop() {

    }

}
