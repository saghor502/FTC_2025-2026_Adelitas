// =============================================================================
// FILE:    Robot.java
// PACKAGE: org.firstinspires.ftc.teamcode.robot
// =============================================================================
// DESCRIPTION:
//   Controls all non-drive mechanisms: launcher flywheel, intake motor,
//   feeder servos, assist servo, and angle servo.
//   Uses a state machine (LaunchState) to sequence the full shoot cycle
//   without blocking the main loop.
//
// HARDWARE OWNERSHIP RULE:
//   This class is the sole owner of all non-drive mechanisms.
//   No OpMode should ever call hardwareMap.get() for launcher, intake, feeders, or servos.
//   All mechanism control must go through the methods in this class.
//
// HARDWARE REQUIRED (set these names in the Driver Hub robot config):
//   "launcher"     — DcMotorEx, flywheel motor (uses velocity PID)
//   "intake"       — DcMotor,   intake roller
//   "left_feeder"  — CRServo,   left  ball feeder
//   "right_feeder" — CRServo,   right ball feeder
//   "assist"       — Servo,     ball assist arm
//   "angle"        — Servo,     launcher angle adjuster
//
// USED BY:
//   - AdelitasTeleopV2 (primary user)
//   - AutoBlue.java    (has its own inline copy — legacy)
//
// HOW TO USE:
//   1. Create once in OpMode.init():
//        robot = new Robot(hardwareMap, telemetry);
//   2. Each loop(), call:
//        robot.launch(shotRequested);   // handles the full shoot sequence
//        robot.setIntakePower(power);   // -1 to 1
//   3. To manually spin the flywheel without shooting:
//        robot.setLauncherVelocity(robot.LAUNCHER_TARGET_VELOCITY);
// =============================================================================
package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {
    private Telemetry robotTelemetry;
    private DcMotorEx launcher;
    private CRServo leftFeeder, rightFeeder;
    private DcMotor intake;
    private Servo assist, angle;
    ElapsedTime feederTimer = new ElapsedTime();
    private LaunchState launchState;
    // Launch sequence states:
    //   IDLE      — waiting for a shot request
    //   SPIN_UP   — flywheel accelerating to target velocity
    //   LAUNCH    — velocity reached, feeders activated
    //   LAUNCHING — feeders running until FEED_TIME_SECONDS elapses
    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }
    public final int LAUNCHER_TARGET_VELOCITY = 1125;
    final double LAUNCHER_MIN_VELOCITY = 1075;
    final double FEED_TIME_SECONDS = 5; //The feeder servos run this long when a shot is requested.
    public final int STOP_SPEED = 0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry){
        launchState = LaunchState.IDLE;

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
        intake = hardwareMap.get(DcMotor.class,"intake");
        assist = hardwareMap.get(Servo.class, "assist");
        angle = hardwareMap.get(Servo.class, "angle");

        rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFeeder.setDirection(DcMotorSimple.Direction.FORWARD);
        assist.setDirection(Servo.Direction.REVERSE);

        launcher.setDirection(DcMotorEx.Direction.REVERSE);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        this.robotTelemetry = telemetry;
    }

    public void setIntakePower(double power){
        intake.setPower(power);
    }

    public void upwardAssist(){
        assist.setPosition(0.4);
    }

    public void downwardsAssist(){
        assist.setPosition(0);
    }

    public void setFeedersPower(double power){
        leftFeeder.setPower(power);
        rightFeeder.setPower(power);
    }

    public void setLauncherVelocity(int setLauncherVelocity){
        launcher.setVelocity(setLauncherVelocity);
    }

    /*private functions*/
    public void launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                    launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                leftFeeder.setPower(FULL_SPEED);
                rightFeeder.setPower(FULL_SPEED);
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    launchState = LaunchState.IDLE;
                    leftFeeder.setPower(STOP_SPEED);
                    rightFeeder.setPower(STOP_SPEED);
                }
                break;
        }
    }
    public LaunchState getLaunchState(){
        return launchState;
    }
    public double getLauncherVelocity(){
        return launcher.getVelocity();
    }

    public void setAngleUp(){
        angle.setPosition(0);
    }

    public void setAngleDown(){
        angle.setPosition(1);
    }
}
