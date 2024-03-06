package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import static org.firstinspires.ftc.teamcode.Utility.expo;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class AsyncRoutines extends Thread{

    private int routine;
    private HardwarePushbot robot;
    private TeleOp360Mov teleOp;
//    private Gamepad gamepad1;
    public boolean run = false;

    public AsyncRoutines(int routine, HardwarePushbot robot, TeleOp360Mov teleOp){
        this.routine = routine;
        this.robot = robot;
        this.teleOp = teleOp;
//        this.gamepad1 = gamepad1;
    }

    public void setRoutine(int routine){
        this.routine = routine;
    }

    // 0 = Y: Arm to Hover Position
    // 1 = X: Arm to Pick Up
    // 2 = A: Arm to High Position
    // 3 = B: Arm to Low Position
    public void run(){
        switch(routine) {
            case 0:
                // Hover Position
                teleOp.residualPower = 0.05;
                robot.claw_rot.setPosition(teleOp.rotHover);
                robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.armUp(robot.armHover);
                break;

            case 1:
                // Grabbing Position
                teleOp.residualPower = 0;
                robot.claw_rot.setPosition(teleOp.rotGrab);
                robot.armDown();
                try {
                    sleep(200);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                robot.claw_right.setPosition(teleOp.closeRight);
                robot.claw_left.setPosition(teleOp.closeLeft);
                break;

            case 2:
                // High Position
                teleOp.residualPower = 0;
                robot.armUp(teleOp.armHigh);
                robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.claw_rot.setPosition(teleOp.rotHigh);
                break;

            case 3:
                // Low Position
                teleOp.residualPower = -0.08;
                robot.armUp(teleOp.armLow);
                robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.claw_rot.setPosition(teleOp.rotLow);
                break;

            case 4:
                //Testing routines

                while (run) {
                    double[] powers =
                            robot.drive(expo(teleOp.gamepad1.left_stick_x, 1.5) / 1.5,
                                    expo(teleOp.gamepad1.left_stick_y, 1.5) / 1.5,
                                    expo(teleOp.gamepad1.right_stick_x, 1.5) / 1.5);


//         Rotation --------------------------------------------------------------------------------
                    if (teleOp.gamepad1.right_bumper || teleOp.gamepad1.right_trigger >= 0.2) {
                        if ((teleOp.gamepad1.right_bumper) && !(teleOp.gamepad1.right_trigger >= 0.2)) {
                            robot.rotate(-0.25 * robot.DRIVE_SPEED);
                        }
//            if (!(gamepad1.right_bumper) && (gamepad1.right_trigger >= 0.2)) {
//                robot.rotate((-gamepad1.right_trigger / 2) * robot.DRIVE_SPEED);
//            }
                    } else if (teleOp.gamepad1.left_bumper || teleOp.gamepad1.left_trigger >= 0.2) {
                        if ((teleOp.gamepad1.left_bumper) && !(teleOp.gamepad1.left_trigger >= 0.2)) {
                            robot.rotate(0.25 * robot.DRIVE_SPEED);
                        }
//            if (!(gamepad1.left_bumper) && (gamepad1.left_trigger >= 0.2)) {
//                robot.rotate((gamepad1.left_trigger / 2) * robot.DRIVE_SPEED);
//            }
                    } else {
                        robot.setVelocities(powers);
                    }
//                        teleOp.telemetry.addData("Front_Left", powers[0]);
//                        teleOp.telemetry.addData("Front_Right", powers[1]);
//                        teleOp.telemetry.addData("Back_Left", powers[2]);
//                        teleOp.telemetry.addData("Back_Right", powers[3]);
                }
        }
    }
}
