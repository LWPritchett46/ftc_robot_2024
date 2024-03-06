package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.dashboard
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import org.firstinspires.ftc.teamcode.TwoWheelTrackingLocalizer;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.util.Angle;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.DriveConstants.kV;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class SampleMecanumDrive extends com.acmerobotics.roadrunner.drive.MecanumDrive {

    //public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(12, 0.002, 0.75);
    //public static PIDCoefficients HEADING_PID = new PIDCoefficients(12, 0.002, 0);
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(12, 0.002, 0.75);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(13, 0.002, 0);
    /*faster try
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(8, 0.001, 2);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(20, 0.004, 0.75);
  */
  // public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(0, 0, 0);

   // public static PIDCoefficients HEADING_PID = new PIDCoefficients(0, 0, 0);

    public static PIDCoefficients TURN_PID = new PIDCoefficients(15, 0.05, .01);
    public static PIDCoefficients FIXTURN_PID = new PIDCoefficients(1.3, 0.002, .1);
    //public static PIDCoefficients Y_PID = new PIDCoefficients(12, 0, 0);

    public  int USE_IMU=0;
    public static double LATERAL_MULTIPLIER = 1.3;////1.1; //was 2.05;
    private IMU gyro;
    public static double VX_WEIGHT = .7;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT =.9;// .8;
    public double speedMult =1; //must be less than 1

    public static int POSE_HISTORY_LIMIT = 100;
    private boolean isIMUread=false;
    private double IMUHeading=0;
    public enum Mode {
        IDLE,
        TURN,
        FIXTURN,
        FOLLOW_TRAJECTORY
    }
    private boolean debug=true;
    private boolean usedashboard=true;
    private FtcDashboard dashboard;
    private TelemetryPacket packet;
    private Canvas fieldOverlay;


    private NanoClock clock;

    public Mode mode;

    private PIDFController turnController;
    private MotionProfile turnProfile;
    private double turnStart;

    private TrajectoryVelocityConstraint velConstraint;
    private TrajectoryAccelerationConstraint accelConstraint;
    private TrajectoryFollower follower;

    private LinkedList<Pose2d> poseHistory;

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;
    private BNO055IMU imu;
    private BNO055IMU imu1;
   // public GyroAnalog gyro;
    private VoltageSensor batteryVoltageSensor;
    private double headingNow;
    private double headingThen;
    private double clockNow;
    private double clockThen;

    private Pose2d lastPoseOnTurn;


    public SampleMecanumDrive(HardwareMap hardwareMap) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);
        //if (useImu) USE_IMU=1;
            dashboard = FtcDashboard.getInstance();
            dashboard.setTelemetryTransmissionInterval(25);


        clock = NanoClock.system();

        mode = Mode.IDLE;
//        gyro = new GyroAnalog(hardwareMap);

        //turnController = new PIDFController(FIXTURN_PID); //Change back to TURN_PID if using main turn
        turnController = new PIDFController(TURN_PID);
        turnController.setInputBounds(0, 2 * Math.PI);

        velConstraint = new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(MAX_ANG_VEL),
                new MecanumVelocityConstraint(MAX_VEL, TRACK_WIDTH)
        ));
        accelConstraint = new ProfileAccelerationConstraint(MAX_ACCEL);
        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                //new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);
                new Pose2d(0.5, 0.5, Math.toRadians(.5)), .5);
        /*follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, Y_PID, HEADING_PID,
                //new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);
                new Pose2d(0.25, 0.25, Math.toRadians(2.0)), 0.5);*/


        poseHistory = new LinkedList<>();

//        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        /*unneeded, now handled in robot class
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        */


        // TODO: adjust the names of the following hardware devices to match your configuration

        // TODO: if your hub is mounted vertically, remap the IMU axes so that the z-axis points
        // upward (normal to the floor) using a command like the following:
        // BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);

        leftFront = hardwareMap.get(DcMotorEx.class, "LF");
        leftRear = hardwareMap.get(DcMotorEx.class, "LR");
        rightRear = hardwareMap.get(DcMotorEx.class, "RR");
        rightFront = hardwareMap.get(DcMotorEx.class, "RF");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        // TODO: reverse any motors using DcMotor.setDirection()
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        //rightRear.setDirection(DcMotor.Direction.REVERSE);
        //leftFront.setDirection(DcMotor.Direction.REVERSE);
        // TODO: if desired, use setLocalizer() to change the localization method
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));
        /*
        if (USE_IMU==1) {
            setLocalizer(new TwoWheelTrackingLocalizer(hardwareMap,this));
        } else {
            setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));

        }*/
    //initLocalizer(hardwareMap,false);
    }


    public void initLocalizer (HardwareMap hardwareMap,boolean useIMU){


        if (useIMU) USE_IMU=1;
        else USE_IMU=0;
        if (USE_IMU==1)
        {

            // gyro = new GyroAnalog(hardwareMap);
            // gyro.gyro.resetDeviceConfigurationForOpMode();
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            imu.initialize(parameters);


            //imu1 = hardwareMap.get(BNO055IMU.class, "imu1");
            //imu1.initialize(parameters);
        }
        if (USE_IMU==1) {
            setLocalizer(new TwoWheelTrackingLocalizer(hardwareMap,this));
        } else {
            setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));

        }


    }


    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, velConstraint, accelConstraint);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, velConstraint, accelConstraint);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, velConstraint, accelConstraint);
    }

    public void fixTurnAsync(double angle) {
        double heading = getPoseEstimate().getHeading();

        lastPoseOnTurn = getPoseEstimate();

        turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(heading, 0, 0, 0),
                new MotionState(heading + angle, 0, 0, 0),
                MAX_ANG_VEL,
                MAX_ANG_ACCEL
        );
        turnController.reset();
        turnController.setTargetPosition(heading+angle); //moved from TURN State - only set target once
        turnController.setTargetVelocity(1.5);
        turnController.setTargetAcceleration(MAX_ANG_ACCEL);
        turnStart = clock.seconds();
        mode = Mode.FIXTURN;
    }

    public void cancelFollowing() {
        mode = Mode.IDLE;
    }

    public void fixTurn(double angle) {
        fixTurnAsync(angle);
        waitForIdle();
    }


    public void turnAsync(double angle) {
        double heading = getPoseEstimate().getHeading();

        lastPoseOnTurn = getPoseEstimate();

        turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(heading, 0, 0, 0),
                new MotionState(heading + angle, 0, 0, 0),
                MAX_ANG_VEL,
                MAX_ANG_ACCEL
        );
        turnController.reset();
        turnStart = clock.seconds();
        mode = Mode.TURN;
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        follower.followTrajectory(trajectory);
        mode = Mode.FOLLOW_TRAJECTORY;
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public Pose2d getLastError() {
        switch (mode) {
            case FOLLOW_TRAJECTORY:
                return follower.getLastError();
            case TURN:
                return new Pose2d(0, 0, turnController.getLastError());
            case FIXTURN:
                return new Pose2d(0, 0, turnController.getLastError());
            case IDLE:
                return new Pose2d();
        }
        throw new AssertionError();
    }

    public void update() {
        if (USE_IMU==1) isIMUread=false;
        updatePoseEstimate();
        Pose2d currentPose = getPoseEstimate();
        Pose2d lastError = getLastError();

        poseHistory.add(currentPose);

        if (POSE_HISTORY_LIMIT > -1 && poseHistory.size() > POSE_HISTORY_LIMIT) {
            poseHistory.removeFirst();
        }

        if (usedashboard) {
            packet = new TelemetryPacket();
            fieldOverlay = packet.fieldOverlay();

            packet.put("mode", mode);

            packet.put("x", currentPose.getX());
            packet.put("y", currentPose.getY());
            packet.put("heading (deg)", Math.toDegrees(currentPose.getHeading()));
         //   packet.put("Gyro voltage", gyro.readGyroVoltage());

            packet.put("xError", lastError.getX());
            packet.put("yError", lastError.getY());
            packet.put("headingError (deg)", Math.toDegrees(lastError.getHeading()));
        }

        switch (mode) {
            case IDLE:
                // do nothing
                break;
            case TURN: {


                double t = clock.seconds() - turnStart;

                MotionState targetState = turnProfile.get(t);
                //turnController.reset();
                turnController.setTargetPosition(targetState.getX());

                MotionState endState = turnProfile.end();

                double error = Angle.normDelta(endState.getX() - currentPose.getHeading());
                if (debug) {
                    System.out.println("TURN_Error " + Math.toDegrees(error));
                    System.out.println("TURN_Heading " + Math.toDegrees(currentPose.getHeading()));
                    System.out.println("TURN_Final Target " + Math.toDegrees(endState.getX()));
                    System.out.println("TURN_Int Target " + Math.toDegrees(targetState.getX()));
                    System.out.println("TURN_time " + t);
                    System.out.println("TURN_duration " + turnProfile.duration());
                }


                //&& error <Math.toRadians(1) is new here
                if ((t >= turnProfile.duration()) && (Math.abs(error) <Math.toRadians(1)||t>turnProfile.duration()+.2))//.5 tol,.25 duration
                {
                    mode = Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                } else {
                    double correction = turnController.update(currentPose.getHeading());
                    double targetOmega;
                    double targetAlpha;
                    if(t<=turnProfile.duration())
                    {
                        targetOmega = targetState.getV();
                        targetAlpha = targetState.getA();
                    }
                    else
                    {
                        targetOmega = Math.signum(correction)*.4;//.5;//.6
                        targetAlpha = 0;
                    }
                  /*
                    if (debug) {
                    System.out.println("TURN_V " + targetOmega);
                    System.out.println("TURN_A " + targetAlpha);
                    System.out.println("TURN_Correction " + correction);
                    System.out.println("TURN_PID error " + Math.toDegrees(turnController.getLastError()));
                    System.out.println("TURN_PID target " + Math.toDegrees(turnController.getTargetPosition()));
                    }*/


                    setDriveSignal(new DriveSignal(new Pose2d(
                            0, 0, targetOmega + correction
                    ), new Pose2d(
                            0, 0, targetAlpha
                    )));

                }
                    Pose2d newPose = lastPoseOnTurn.copy(lastPoseOnTurn.getX(), lastPoseOnTurn.getY(), targetState.getX());
                if (usedashboard) {
                    fieldOverlay.setStroke("#4CAF50");
                    org.firstinspires.ftc.teamcode.util.DashboardUtil.drawRobot(fieldOverlay, newPose);
                }

                break;
            }
            case FIXTURN: {


                double correction = turnController.update(currentPose.getHeading());

              double error = Angle.normDelta(turnController.getTargetPosition() - currentPose.getHeading()); //don't need, debugging only
                double PIDerror=turnController.getLastError();
                if (debug) {
                    System.out.println("TURN_Error " + Math.toDegrees(error));
                    System.out.println("TURN_Error_FromPID " + Math.toDegrees(PIDerror));
                    System.out.println("TURN_Heading before turn" + Math.toDegrees(currentPose.getHeading()));
                    System.out.println("TURN_Final Target " + Math.toDegrees(turnController.getTargetPosition()));
                    System.out.println("TURN_PID Vel " + correction);

                  //  System.out.println("TURN_time " + t);
                   // System.out.println("TURN_duration " + turnProfile.duration());
                }
                Pose2d newPose = lastPoseOnTurn.copy(lastPoseOnTurn.getX(), lastPoseOnTurn.getY(), currentPose.getHeading());
                if (usedashboard) {
                    fieldOverlay.setStroke("#4CAF50");
                    org.firstinspires.ftc.teamcode.util.DashboardUtil.drawRobot(fieldOverlay, newPose);
                }

                //&& error <Math.toRadians(1) is new here
                //if (t >= turnProfile.duration() && (Math.abs(error) <Math.toRadians(1)||t>turnProfile.duration()+.05))//.5 tol,.25 duration
                if (Math.abs(PIDerror)<Math.toRadians(0.5))
                    {
                    mode = Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                } else {

                    setDriveSignal(new DriveSignal(new Pose2d(0, 0, correction),
                            new Pose2d(0, 0, 0)));

                }


                break;
            }
            case FOLLOW_TRAJECTORY: {
                setDriveSignal(follower.update(currentPose, getPoseVelocity()));

                Trajectory trajectory = follower.getTrajectory();

                if (usedashboard) {
                    fieldOverlay.setStrokeWidth(1);
                    fieldOverlay.setStroke("#4CAF50");
                    org.firstinspires.ftc.teamcode.util.DashboardUtil.drawSampledPath(fieldOverlay, trajectory.getPath());
                }
                double t = follower.elapsedTime();

                if (usedashboard) {
                    org.firstinspires.ftc.teamcode.util.DashboardUtil.drawRobot(fieldOverlay, trajectory.get(t));

                    fieldOverlay.setStroke("#3F51B5");
                    org.firstinspires.ftc.teamcode.util.DashboardUtil.drawPoseHistory(fieldOverlay, poseHistory);
                }
                if (!follower.isFollowing()) {
                    mode = Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                }

                break;
            }
        }

        if (usedashboard) {
            fieldOverlay.setStroke("#3F51B5");
            org.firstinspires.ftc.teamcode.util.DashboardUtil.drawRobot(fieldOverlay, currentPose);

            dashboard.sendTelemetryPacket(packet);
        }
        /*
        if (USE_IMU==1) {
            headingThen = headingNow;
            clockThen = clockNow;
            headingNow = getRawExternalHeading();
            clockNow = clock.seconds();
        }

         */

    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy()) {
            update();
        }
    }

    public boolean isBusy() {
        return mode != Mode.IDLE;
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );
        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        double head;
        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());


            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX()* speedMult,
                    VY_WEIGHT * drivePower.getY()* speedMult,
                    OMEGA_WEIGHT * drivePower.getHeading()* speedMult
            ).div(denom);
        }

        if (vel.getHeading()> 0)
        {head= Range.clip(vel.getHeading(),.25,1);}
        else if (vel.getHeading()<0)
        { head =Range.clip(vel.getHeading(),-1,-.25);}
        else head=0;
        Pose2d newVel =new Pose2d(vel.getX(),vel.getY(),head);
        setDrivePower(newVel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
        //if (debug)
          //  System.out.println("TURN_Motor Power " + v);
    }

    @Override
    public double getRawExternalHeading() {
       /*if (USE_IMU==1) {
           //double angle1 = gyro.readGyro();
           double angle1= (TwoWheelTrackingLocalizer)getLocalizer().getHeading();
          //double angle1 = imu.getAngularOrientation().firstAngle;
          /* double angle2 = imu1.getAngularOrientation().firstAngle;
           if(angle1 < Math.toRadians(180))
           {
               angle1 += Math.toRadians(360);
           }
           if(angle2 < Math.toRadians(180))
           {
               angle2 += Math.toRadians(360);
           }
            return ((angle1+angle2)/2) % Math.toRadians(360);
           return angle1;
        } else{
            return 0;
        }
*/
        if (USE_IMU==1) {
           // System.out.println("gyro "+Math.toDegrees(gyro.readGyro()));
            //return gyro.readGyro();
           if (isIMUread) {
                return IMUHeading;
            } else {
               isIMUread=true;
                IMUHeading= imu.getAngularOrientation().firstAngle;
                return IMUHeading;
         //       return  imu.getAngularOrientation().firstAngle;

            }
        }
        else return 0;

    }
/*
    @Override
    public Double getExternalHeadingVelocity() {
        // TODO: This must be changed to match your configuration
        //                           | Z axis
        //                           |
        //     (Motor Port Side)     |   / X axis
        //                       ____|__/____
        //          Y axis     / *   | /    /|   (IO Side)
        //          _________ /______|/    //      I2C
        //                   /___________ //     Digital
        //                  |____________|/      Analog
        //
        //                 (Servo Port Side)
        //
        // The positive x axis points toward the USB port(s)
        //
        // Adjust the axis rotation rate as necessary
        // Rotate about the z axis is the default assuming your REV Hub/Control Hub is laying
        // flat on a surface

       // return (double) imu.getAngularVelocity().zRotationRate;
        if (USE_IMU==1)
        return (headingNow-headingThen)/(clockNow-clockThen);
        else return null;
    }
*/
}

