package org.firstinspires.ftc.teamcode.resources;/** Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **/

import static android.os.SystemClock.sleep;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwarePushbotCam {


    /* Public OpMode members. */
    public DcMotorEx left_front = null;
    public DcMotorEx right_front = null;
    public DcMotorEx left_back = null;
    public DcMotorEx right_back = null;

    public IMU imu;


    // Hardware-specific constants
    static final double correction = 2.5;
    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 0.4;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_CM = 10.16;     // For figuring circumference
    static final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * Math.PI) * correction;
    static final double DRIVE_SPEED = 1500;
    static final double TURN_SPEED_MODIFIER = 0.5;

    static final double ANGLE_TOLERANCE = 3.0;


    // local OpMode members.
    HardwareMap hwMap;


    // Constructor
    public HardwarePushbotCam() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
//        left_front = hwMap.get(DcMotorEx.class, "leftfront_wheel");
//        right_front = hwMap.get(DcMotorEx.class, "rightfront_wheel");
//        left_back = hwMap.get(DcMotorEx.class, "leftback_wheel");
//        right_back = hwMap.get(DcMotorEx.class, "rightback_wheel");

        //right_back.setDirection(DcMotor.Direction.REVERSE);
//        left_front.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set all motors to zero velocity
//        left_front.setVelocity(0);
//        right_front.setVelocity(0);
//        left_back.setVelocity(0);
//        right_back.setVelocity(0);


        // Initialize and reset IMU
        imu = hwMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
    }

    // [left_front, right_front, left_back, right_back]
    public void setVelocities(double[] velocities) {

        left_front.setVelocity(velocities[0]);
        right_front.setVelocity(velocities[1]);
        left_back.setVelocity(velocities[2]);
        right_back.setVelocity(velocities[3]);
    }

    public void rotate(double velocity) {

        left_front.setVelocity(-velocity);
        right_front.setVelocity(velocity);
        left_back.setVelocity(-velocity);
        right_back.setVelocity(velocity);
    }

    public void rotateTo(double target, double velocityFactor) {

        double angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        while (Math.abs(target - angle) >= ANGLE_TOLERANCE) {

            angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double velocity = velocityFactor * Math.pow(target - angle, 2);

            this.left_front.setVelocity(-velocity);
            this.right_front.setVelocity(velocity);
            this.left_back.setVelocity(-velocity);
            this.right_back.setVelocity(velocity);

        }
    }


    public double[] drive(float leftStickX, float leftStickY, float rightStickX){

        double r = Math.hypot(leftStickX, leftStickY);
        rightStickX *= TURN_SPEED_MODIFIER;
        double robotAngle = Math.atan2(-leftStickY, leftStickX) - Math.PI / 4;
        double v1 = r * Math.cos(robotAngle);// + (double) leftStickX;
        double v2 = r * Math.sin(robotAngle);// - (double) leftStickX;
        double v3 = r * Math.sin(robotAngle);// + (double) leftStickX;
        double v4 = r * Math.cos(robotAngle);// - (double) leftStickX;

        v1 += rightStickX;
        v2 -= rightStickX;
        v3 += rightStickX;
        v4 -= rightStickX;

        return new double[]{v1 * DRIVE_SPEED, v2 * DRIVE_SPEED, v3 * DRIVE_SPEED, v4 * DRIVE_SPEED};
    }

    public void brake() {

        left_front.setVelocity(0);
        right_front.setVelocity(0);
        left_back.setVelocity(0);
        right_back.setVelocity(0);
    }

    public void moveTo(double angle, int distanceCM, double velocity) {

        angle -= Math.PI/4;

        double v1 = velocity * Math.cos(angle);
        double v2 = velocity * Math.sin(angle);
        // Excluded v3 and v4 because v3 = v2 and v4 = v1

        int p1 = (int)(Math.round(Math.cos(angle) * distanceCM));
        int p2 = (int)(Math.round(Math.sin(angle) * distanceCM));
        // Excluded p3 and p4 because p3 = p2 and p4 = p1

        this.left_front.setTargetPosition(p1);
        this.right_front.setTargetPosition(p2);
        this.left_back.setTargetPosition(p2);
        this.right_back.setTargetPosition(p1);

        this.left_front.setPower(v1);
        this.right_front.setPower(v2);
        this.left_back.setPower(v2);
        this.right_back.setPower(v1);

        this.left_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.right_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.left_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.right_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void move(double angle, int velocity) {

        angle -= Math.PI / 4;

        double v1 = velocity * Math.cos(angle);
        double v2 = velocity * Math.sin(angle);
        double v3 = velocity * Math.sin(angle);
        double v4 = velocity * Math.cos(angle);

        if (this.left_front.getVelocity() != v1) {
            this.left_front.setVelocity(v1);
        }
        if (this.right_front.getVelocity() != v2) {
            this.right_front.setVelocity(v2);
        }
        if (this.left_back.getVelocity() != v3) {
            this.left_back.setVelocity(v3);
        }
        if (this.right_back.getVelocity() != v4) {
            this.right_back.setVelocity(v4);
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

    public double getAngularPos(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void resetIMU(){
        imu.resetYaw();
    }
}