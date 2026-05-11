// =============================================================================
// FILE:    Chassis.java
// PACKAGE: org.firstinspires.ftc.teamcode.robot
// =============================================================================
// DESCRIPTION:
//   Controls the 4-wheel mecanum drivetrain.
//   Handles motor setup, direction, power normalization, and all drive modes.
//   This is the lowest-level movement class — every other class that needs
//   to move the robot must go through Chassis.
//
// HARDWARE OWNERSHIP RULE:
//   This class is the sole owner of the drive motors.
//   No OpMode (autonomous or TeleOp) should ever call hardwareMap.get() for a drive motor.
//   All movement must go through the methods in this class.
//
// HARDWARE REQUIRED (set these names in the Driver Hub robot config):
//   "lf" — front-left  drive motor
//   "rf" — front-right drive motor
//   "lb" — back-left   drive motor
//   "rb" — back-right  drive motor
//
// USED BY:
//   - Odometry.java    (calls mecanumDrive / stopChasis during navigation)
//   - ImuTurning.java  (calls turn / stopChasis during PID turns)
//   - AdelitasTeleopV2 (calls mecanumDrive every loop)
//   - Example autonomous OpModes
//
// HOW TO USE:
//   1. Create once in OpMode.init():
//        chassis = new Chassis(hardwareMap, telemetry);
//   2. Drive the robot each loop:
//        chassis.mecanumDrive(forward, strafe, rotate, slowMode);
//   3. Stop when done:
//        chassis.stopChasis();
// =============================================================================
package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Chassis {
    private Telemetry chassisTelemetry;
    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;

    // Internal power values kept as fields so they can be read for telemetry if needed
    private double leftFrontPower, rightFrontPower, leftBackPower, rightBackPower;

    // Initializes all 4 drive motors and sets their directions.
    // Call once in OpMode.init().
    public Chassis(HardwareMap hardwareMap, Telemetry telemetry){
        leftFrontDrive = hardwareMap.get(DcMotor.class, "lf");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rf");
        leftBackDrive = hardwareMap.get(DcMotor.class, "lb");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rb");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        this.chassisTelemetry = telemetry;
    }

    // Full mecanum drive. forward/strafe/rotate are in range [-1, 1].
    // Powers are normalized so no motor ever exceeds 1.0.
    // Set slow = true to cap all motors at 50% (useful for precise TeleOp movement).
    public void mecanumDrive(double forward, double strafe, double rotate, boolean slow){
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        leftFrontPower = (forward + strafe + rotate) / denominator;
        rightFrontPower = (forward - strafe - rotate) / denominator;
        leftBackPower = (forward - strafe + rotate) / denominator;
        rightBackPower = (forward + strafe - rotate) / denominator;

        if(slow){
            leftFrontDrive.setPower(leftFrontPower * 0.5);
            rightFrontDrive.setPower(rightFrontPower * 0.5);
            leftBackDrive.setPower(leftBackPower * 0.5);
            rightBackDrive.setPower(rightBackPower * 0.5);
        }else{
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
        }
    }

    // Drives all 4 motors at the same power. Positive = forward, negative = backward.
    public void move(double power){
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);
    }
    // Strafes (slides) the robot sideways. Positive = right, negative = left.
    public void strafe(double power){
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(-power);
        leftBackDrive.setPower(-power);
        rightBackDrive.setPower(power);
    }
    // Turns the robot in place. Positive = clockwise, negative = counterclockwise.
    // Used by ImuTurning for PID-controlled rotation.
    public void turn(double power){
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(-power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(-power);
    }

    // Cuts power to all motors immediately. Call at the end of any movement routine.
    public void stopChasis(){
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
    // Encoder tick readers — used by Odometry for pose tracking.
    public int getLF() { return leftFrontDrive.getCurrentPosition(); }
    public int getRF() { return rightFrontDrive.getCurrentPosition(); }
    public int getLB() { return leftBackDrive.getCurrentPosition(); }
    public int getRB() { return rightBackDrive.getCurrentPosition(); }

    // Resets all encoder counts to 0 and puts motors back in RUN_WITHOUT_ENCODER mode.
    // Call before any odometry session starts.
    public void resetEncoders() {
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
