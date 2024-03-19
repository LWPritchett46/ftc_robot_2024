package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import static org.firstinspires.ftc.teamcode.util.Utility.clamp;
import static org.firstinspires.ftc.teamcode.util.Utility.wrapIMUDeg;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.Utility;

import kotlin.jvm.functions.Function2;


public class HardwarePushbot {

    // Motors and Sensors
    public DcMotorEx left_front;
    public DcMotorEx right_front;
    public DcMotorEx left_back;
    public DcMotorEx right_back;

    public DcMotorEx arm;
    public DcMotorEx intake;
    public DcMotorEx lift;

    public Servo claw_rot;
    public Servo claw_right;
    public Servo claw_left;
    public Servo launcher;
    public Servo intake_rot;
    public Servo left_wiper;

    public IMU imu;

    //double residualPower = 0;


    // Constants
    static final double correction = 2.5;
    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 0.4;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_CM = 10.16;     // For figuring circumference
    static final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * Math.PI) * correction;
    static final double DRIVE_SPEED = 2700;
    static final double TURN_SPEED_MODIFIER = 0.7;
    static final double ANGLE_TOLERANCE = 1;

    static final double joystickExp = 1.5;

    public static final int playOffset = -100;

    public int armHover = 1000 + playOffset;

    public TeleOp360Mov.MovingState movingState = TeleOp360Mov.MovingState.NO_MOVE;


    public boolean isHolding = false;
    public int holdPos = 0;

    double kP = 0.5;
    double kI = 0.5;
    double kD = 0.5;
    double kG = 0.5;

    public PIDCoefficients armCoeffs = new PIDCoefficients(kP, kI, kD);
    public PIDFController armController = new PIDFController(armCoeffs);

    public MotionProfile armProfile1 = MotionProfileGenerator.generateSimpleMotionProfile(
            new MotionState(0, 0, 0),
            new MotionState(60, 60, 0),
            25, 40, 100
    );

    public MotionProfile armProfile2 = MotionProfileGenerator.generateSimpleMotionProfile(
            new MotionState(60, 60, 0),
            new MotionState(100, 0, 0),
            25, 20, 100
    );

    HardwareMap hwMap;

    // Initialize standard Hardware interfaces
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Drivetrain Motors
        left_front = hwMap.get(DcMotorEx.class, "leftfront_wheel");
        right_front = hwMap.get(DcMotorEx.class, "rightfront_wheel");
        left_back = hwMap.get(DcMotorEx.class, "leftback_wheel");
        right_back = hwMap.get(DcMotorEx.class, "rightback_wheel");

        // Functionality Motors
        intake = hwMap.get(DcMotorEx.class, "intake");
        arm = hwMap.get(DcMotorEx.class, "arm");
        lift = hwMap.get(DcMotorEx.class, "lift");

        // Servos
        launcher = hwMap.get(Servo.class, "launcher");
        claw_rot = hwMap.get(Servo.class, "claw_rot");
        claw_right = hwMap.get(Servo.class, "claw_right");
        claw_left = hwMap.get(Servo.class, "claw_left");
        intake_rot = hwMap.get(Servo.class, "intake_rot");

        left_wiper = hwMap.get(Servo.class, "left_wiper");




        // Reverse needed motors and start encoders at zero
        right_front.setDirection(DcMotor.Direction.REVERSE);
        left_back.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);


        //intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set all motors to zero velocity (Redundant asf)
        left_front.setVelocity(0);
        right_front.setVelocity(0);
        left_back.setVelocity(0);
        right_back.setVelocity(0);
        arm.setVelocity(0);

        // Initialize and reset IMU
        imu = hwMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
    }


    // Key: velocities = [left_front, right_front, left_back, right_back]
    public void setVelocities(double[] velocities) {

        left_front.setVelocity(velocities[0]);
        right_front.setVelocity(velocities[1]);
        left_back.setVelocity(velocities[2]);
        right_back.setVelocity(velocities[3]);
    }

    public void setModes(DcMotor.RunMode mode) {

        left_front.setMode(mode);
        right_front.setMode(mode);
        left_back.setMode(mode);
        right_back.setMode(mode);
    }

    public void rotate(double velocity) {

        left_front.setVelocity(-velocity);
        right_front.setVelocity(velocity);
        left_back.setVelocity(-velocity);
        right_back.setVelocity(velocity);
    }


    // target: target angle in degrees
    // maxVel: maximum allowed velocity
    // initVel: fraction of maxVel to be used as the initial and final velocity
    public void rotateTo(double target, double maxVel, double initVel, boolean isClockwise, double pow) {


        double angle = wrapIMUDeg(this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        boolean throughZero;
        double diff;
        double factor;

        throughZero = isClockwise && target > angle;


        if (throughZero) {
            diff = 360 - Math.abs(target - angle);
        } else {
            diff = Math.abs(target - angle);
        }

        double midpoint = diff/2;

        midpoint *= .90;

        while (Math.abs(diff) >= ANGLE_TOLERANCE) {

            if (!Double.isNaN(wrapIMUDeg(this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)))) {
                angle = wrapIMUDeg(this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            }

            double velocity = maxVel
                    * ((1 - initVel)
                    * (Math.pow(1 - clamp(Math.abs((diff - midpoint) / (midpoint)), 1, 0), pow))
                    + initVel);

            if (isClockwise && target > angle) {
                throughZero = true;
            } else {
                throughZero = !isClockwise && target < angle;
            }


            if (throughZero) {
                diff = 360 - Math.abs(target - angle);
            } else {
                diff = Math.abs(target - angle);
            }

            if(diff <= ANGLE_TOLERANCE){
                this.brake();
                continue;
            }

            if (isClockwise) {
                velocity *= -1;
            }

            this.left_front.setVelocity(-velocity);
            this.right_front.setVelocity(velocity);
            this.left_back.setVelocity(-velocity);
            this.right_back.setVelocity(velocity);
        }
        this.brake();
    }


    public double[] drive(double leftStickX, double leftStickY, double rightStickX){

        double r = Math.hypot(leftStickX, leftStickY);
        rightStickX *= TURN_SPEED_MODIFIER;


        double robotAngle = Math.atan2(-leftStickY, leftStickX)
                - Math.PI / 4;
                //- this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double v1 = r * Math.cos(robotAngle) + (double) rightStickX;
        double v2 = r * Math.sin(robotAngle) - (double) rightStickX;
        double v3 = r * Math.sin(robotAngle) + (double) rightStickX;
        double v4 = r * Math.cos(robotAngle) - (double) rightStickX;

        double[] velocities = new double[]{v1, v2, v3, v4};
        double maxVal = Utility.max(velocities);

        if (Math.abs(maxVal) > 1) {
            for (int i = 0; i < velocities.length; i++) {
                velocities[i] /= maxVal;
                velocities[i] *= DRIVE_SPEED;
            }
        }
        else {
            for (int i = 0; i < velocities.length; i++) {
                velocities[i] *= DRIVE_SPEED;
            }
        }

        return velocities;
    }


    public void brake() {

        left_front.setVelocity(0);
        right_front.setVelocity(0);
        left_back.setVelocity(0);
        right_back.setVelocity(0);
    }


    // WIP
    public void moveTo(double angle, int ticks, double maxVelocity, double pow, Telemetry telemetry) {


        telemetry.addLine("Made it into moveTo");
        telemetry.update();

        this.setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.setModes(DcMotor.RunMode.RUN_USING_ENCODER);


        double tolerance = 30;
        double initVelocity = 0.15;

        angle += Math.PI/4;
//        angle -= this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double v1 = maxVelocity * Math.cos(angle);
        double v2 = maxVelocity * Math.sin(angle);
        // Excluded v3 and v4 because v3 = v2 and v4 = v1

        int p1 = (int)(Math.round(Math.cos(angle) * ticks));
        int p2 = (int)(Math.round(Math.sin(angle) * ticks));
        // Excluded p3 and p4 because p3 = p2 and p4 = p1

//        this.left_front.setTargetPosition(p1);
//        this.right_front.setTargetPosition(p2);
//        this.left_back.setTargetPosition(p2);
//        this.right_back.setTargetPosition(p1);

        boolean p1overCurrent = p1 > this.left_front.getCurrentPosition();
        boolean p2overCurrent = p2 > this.right_front.getCurrentPosition();

        double midpoint1 = p1/2.0;
        midpoint1 *= 0.9;
        double midpoint2 = p2/2.0;
        midpoint2 *= 0.9;

//        this.left_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        this.right_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        this.left_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        this.right_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (Math.abs(p1 - this.left_front.getCurrentPosition()) >= tolerance ||
                Math.abs(p2 - this.right_front.getCurrentPosition()) >= tolerance) {

            double factor1 = (1 - initVelocity) * (Math.pow(clamp(1 - Math.abs((this.left_front.getCurrentPosition() - midpoint1) / midpoint1), 1, 0), pow) + initVelocity);
            double factor2 = (1 - initVelocity) * (Math.pow(clamp(1 - Math.abs((this.right_front.getCurrentPosition() - midpoint2) / midpoint2), 1, 0), pow) + initVelocity);
            //double factor =(factor2 + factor1)/2;


            if (Math.abs(p1 - this.left_front.getCurrentPosition()) >= tolerance){
                if (p1overCurrent == p1 > this.left_front.getCurrentPosition()) {
                    this.left_front.setVelocity(v1 * factor1);
                    this.right_back.setVelocity(v1 * factor1);
                }
                else{
                    this.left_front.setVelocity(v1 * factor1 * -1);
                    this.right_back.setVelocity(v1 * factor1 * -1);
                }
            }
            else {
                this.left_front.setVelocity(0);
                this.right_back.setVelocity(0);
            }


            if (Math.abs(p2 - this.right_front.getCurrentPosition()) >= tolerance) {
                if (p2overCurrent == p2 > this.right_front.getCurrentPosition()) {
                    this.right_front.setVelocity(v2 * factor2);
                    this.left_back.setVelocity(v2 * factor2);
                }
                else{
                    this.right_front.setVelocity(v2 * factor2 * -1);
                    this.left_back.setVelocity(v2 * factor2 * -1);
                }
            }
            else{
                this.right_front.setVelocity(0);
                this.left_back.setVelocity(0);
            }


            telemetry.addData("Current left_front position", this.left_front.getCurrentPosition());
            telemetry.addData("Current right_front position", this.right_front.getCurrentPosition());
//            telemetry.addData("Current left_back position", this.left_back.getCurrentPosition());
//            telemetry.addData("Current right_back position", this.right_back.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("Target left_front position", p1);
            telemetry.addData("Target right_front position", p2);
            telemetry.addLine();
            telemetry.addData("Factor 1", factor1);
            telemetry.addData("Factor 2", factor2);
            telemetry.addLine();
            telemetry.addData("Diff 1", Math.abs(p1 - this.left_front.getCurrentPosition()));
            telemetry.addData("Diff 2", Math.abs(p2 - this.right_front.getCurrentPosition()));

            telemetry.update();
        }

        this.brake();
    }


    public double[] move(double direction, double translation, double rotation, double factor) {

        direction += Math.PI / 4;
        direction -= this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        int maxVelocity = 1300;

        double v1 = (Math.abs(factor) * translation * Math.cos(direction)) - (factor * rotation * translation);
        double v2 = (Math.abs(factor) * translation * Math.sin(direction)) + (factor * rotation * translation);
        double v3 = (Math.abs(factor) * translation * Math.sin(direction)) - (factor * rotation * translation);
        double v4 = (Math.abs(factor) * translation * Math.cos(direction)) + (factor * rotation * translation);

        double[] velocities = new double[]{v1, v2, v3, v4};
        double maxVal = Utility.max(velocities);

        if (Math.abs(maxVal) > 1) {
            for (int i = 0; i < velocities.length; i++) {
                velocities[i] /= maxVal;
                velocities[i] *= maxVelocity;
            }
        }
        else {
            for (int i = 0; i < velocities.length; i++) {
                velocities[i] *= maxVelocity;
            }
        }
        return velocities;
//
//        if (this.left_front.getVelocity() != velocities[0]) {
//            this.left_front.setVelocity(velocities[0]);
//        }
//        if (this.right_front.getVelocity() != velocities[1]) {
//            this.right_front.setVelocity(velocities[1]);
//        }
//        if (this.left_back.getVelocity() != velocities[2]) {
//            this.left_back.setVelocity(velocities[2]);
//        }
//        if (this.right_back.getVelocity() != velocities[3]) {
//            this.right_back.setVelocity(velocities[3]);
//        }
    }

    public void moveRotateTo(int target, double direction, double initVel, boolean isClockwise, double translationW, double rotationW, Telemetry telemetry){
        double angle = wrapIMUDeg(this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        boolean throughZero;
        double diff;
        double oldDiff;
        double factor;

        throughZero = isClockwise && target > angle;


        if (throughZero) {
            diff = 360 - Math.abs(target - angle);
        } else {
            diff = Math.abs(target - angle);
        }

        double midpoint = diff/2;
        oldDiff = diff;

        midpoint *= .90;

//        telemetry.addData("Diff b4 if", diff);
//        telemetry.addLine();
//
//        telemetry.addData("IMU Angle", robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
//        telemetry.addData("Wrapped Angle", angle);
//        telemetry.addData("Diff", diff);
//
//        telemetry.update();

        while (diff >= ANGLE_TOLERANCE) {


            if (!Double.isNaN(wrapIMUDeg(this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)))) {
                angle = wrapIMUDeg(this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            }


            if (isClockwise && target > angle) {
                throughZero = true;
            } else {
                throughZero = !isClockwise && target < angle;
            }


            if (throughZero) {
                diff = 360 - Math.abs(target - angle);
            } else {
                diff = Math.abs(target - angle);
            }

            if(diff <= ANGLE_TOLERANCE){
                this.brake();
                continue;
            }


            factor = (1 - initVel)
                    * (Math.pow(1 - clamp(Math.abs((diff - midpoint) / (midpoint)), 1, 0), 1.6))
                    + initVel;


            if (isClockwise) {
                factor *= -1;
            }
//            if (oldDiff - ANGLE_TOLERANCE > diff){
//                factor *= -1;
//            }

            oldDiff = diff;


            this.setVelocities(this.move(direction, translationW, rotationW, factor));

            telemetry.addData("Factor", factor);
            telemetry.addLine();
            telemetry.addData("Target", target);
            telemetry.addData("Raw angle", this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("Wrapped angle", wrapIMUDeg(this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)));
            telemetry.addData("Through Zero", throughZero);
            telemetry.addLine();
            telemetry.addData("MidPoint", midpoint);
            telemetry.addData("<1", Math.abs((diff - midpoint)) / midpoint);
            telemetry.addLine();
            telemetry.addData("Diff", diff);


            telemetry.update();
        }
    }



    public void encoderDrive(double speed, double leftCM, double rightCM, int isSide, int timeout) {
        int newLeftTarget;
        int newRightTarget;
        int newLeftTargetBack;
        int newRightTargetBack;

        if (isSide == 1) {
            newLeftTarget = this.left_front.getCurrentPosition() + (int) (-leftCM * COUNTS_PER_CM);
            newRightTarget = this.right_front.getCurrentPosition() + (int) (rightCM * COUNTS_PER_CM);
            newLeftTargetBack = this.left_back.getCurrentPosition() + (int) (leftCM * COUNTS_PER_CM);
            newRightTargetBack = this.right_back.getCurrentPosition() + (int) (-rightCM * COUNTS_PER_CM);
        } else if (isSide == 2) {
            newLeftTarget = this.left_front.getCurrentPosition() + (int) (leftCM * COUNTS_PER_CM);
            newRightTarget = this.right_front.getCurrentPosition() + (int) (-rightCM * COUNTS_PER_CM);
            newLeftTargetBack = this.left_back.getCurrentPosition() + (int) (-leftCM * COUNTS_PER_CM);
            newRightTargetBack = this.right_back.getCurrentPosition() + (int) (rightCM * COUNTS_PER_CM);
        } else {
            newLeftTarget = this.left_front.getCurrentPosition() + (int) (leftCM * COUNTS_PER_CM);
            newRightTarget = this.right_front.getCurrentPosition() + (int) (rightCM * COUNTS_PER_CM);
            newLeftTargetBack = this.left_back.getCurrentPosition() + (int) (leftCM * COUNTS_PER_CM);
            newRightTargetBack = this.right_back.getCurrentPosition() + (int) (rightCM * COUNTS_PER_CM);
        }

        // Determine new target position, and pass to motor controller
        this.left_front.setTargetPosition(newLeftTarget);
        this.right_front.setTargetPosition(newRightTarget);
        this.left_back.setTargetPosition(newLeftTargetBack);
        this.right_back.setTargetPosition(newRightTargetBack);

        // Turn On RUN_TO_POSITION
        this.left_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.right_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.left_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.right_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        this.left_front.setPower(Math.abs(speed));
        this.right_front.setPower(Math.abs(speed));
        this.left_back.setPower(Math.abs(speed));
        this.right_back.setPower(Math.abs(speed));

        sleep(timeout);

        this.left_front.setPower(0);
        this.right_front.setPower(0);
        this.left_back.setPower(0);
        this.right_back.setPower(0);

        // Turn off RUN_TO_POSITION
        this.left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void armUp(int pos){

        this.isHolding = false;

        //Move arm to high position
        this.arm.setTargetPosition(pos);
        this.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.arm.setPower(0.8);
        while (this.arm.isBusy()) {
            customWait(50);
        }
        this.arm.setPower(0);
        this.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Start HoldPos
        this.holdPos = pos;
        this.isHolding = true;
        this.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public void armDown() {
        //Open claw
//        this.claw_right.setPosition(1);
//        this.claw_left.setPosition(0.1);
        isHolding = false;
        this.arm.setTargetPosition(0);
        this.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.arm.setPower(0.7);
        while (this.arm.isBusy()) {
            customWait(50);
        }
        this.arm.setPower(0);
        this.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }


    public void holdPos(double residualPower){
        this.arm.setTargetPosition(holdPos);
        this.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.arm.setPower(0.7);

        // Commented out to try to fix input delay
//        while (this.arm.isBusy()) {
//            sleep(50);
//        }
        this.arm.setPower(residualPower);
        this.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void unfoldingRoutine() {

        this.intake_rot.setPosition(TeleOp360Mov.intakeOpen);
        sleep(1000);
        this.intake_rot.setPosition(TeleOp360Mov.intakeOpen);
        sleep(1000);
        this.intake_rot.getController().pwmDisable();
        sleep(1000);

//        this.claw_rot.setPosition(TeleOp360Mov.rotHigh);
//        sleep(200);
        this.claw_rot.setPosition(TeleOp360Mov.rotHigh);

        sleep(200);

//        this.claw_left.setPosition(TeleOp360Mov.openLeft);
//        this.claw_right.setPosition(TeleOp360Mov.openRight);
//        sleep(100);
        this.claw_left.setPosition(TeleOp360Mov.openLeft);
        this.claw_right.setPosition(TeleOp360Mov.openRight);

        //sleep(100);
        this.armUp(1220);
        this.claw_rot.setPosition(TeleOp360Mov.rotHover);
        sleep(300);
        this.claw_rot.setPosition(TeleOp360Mov.rotHover);
        this.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.armUp(this.armHover);
    }

    public void customWait(long time) {

        sleep(time);
//        ElapsedTime elapsedTime = new ElapsedTime();
//
//        elapsedTime.startTime();
//        while (elapsedTime.time(TimeUnit.MILLISECONDS) < time){
//            continue;
//        }
    }

    public void tick(){
        switch (movingState){
            case MOVE_UP:
                if (arm.getCurrentPosition() >= TeleOp360Mov.lowerLimit + 500){
                    arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    arm.setPower(0);
                    movingState = TeleOp360Mov.MovingState.NO_MOVE;
                }
                break;
            case MOVE_DOWN:
                if (arm.getCurrentPosition() <= TeleOp360Mov.lowerLimit - 500){
                    arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    arm.setPower(0);
                    movingState = TeleOp360Mov.MovingState.NO_MOVE;
                }
                break;
            case NO_MOVE:
                break;
        }
    }

    // Write Autonomous Routine Stages Here!
}