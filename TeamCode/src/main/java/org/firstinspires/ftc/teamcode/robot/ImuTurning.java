// =============================================================================
// FILE:    ImuTurning.java
// PACKAGE: org.firstinspires.ftc.teamcode.robot
// =============================================================================
// DESCRIPTION:
//   Standalone PID controller for rotating the robot to a target heading
//   using the built-in IMU (gyroscope) on the Control Hub.
//   Use this for point turns in TeleOp or simple autonomous steps that
//   don't need full BFS path navigation.
//   For heading alignment during navigation, Odometry.java handles it internally.
//
// HARDWARE OWNERSHIP RULE:
//   This class is the sole owner of the IMU.
//   No OpMode or other class should ever call hardwareMap.get() for the IMU.
//   Odometry.java reads heading through this class — not through its own IMU instance.
//
// HARDWARE REQUIRED:
//   "imu 1" — built-in IMU on the Control Hub (no extra wiring needed)
//   Chassis instance — for sending turn power to the drive motors
//
// USED BY:
//   - ImuTurningExample.java  (usage reference)
//   - SimpleRobotMovementTest (legacy, uses old single-call API)
//
// HOW TO USE:
//   1. Create once in OpMode.init():
//        imuTurning = new ImuTurning(hardwareMap, telemetry, chassis);
//   2. Start a turn (call once per turn, NOT every loop):
//        imuTurning.startTurn(90);   // target heading in degrees
//   3. Execute the turn (call every loop until it returns false):
//        while (opModeIsActive() && imuTurning.update()) {
//            telemetry.update();
//        }
//   4. To chain turns, call startTurn() again with the next target.
//
// TUNING — see TODO comments on the PID constants below.
// =============================================================================
package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class ImuTurning {

    // =========================================================
    // PID CONSTANTS
    // Tuning order:
    //   1. Set kI = 0, kD = 0. Raise kP until the robot reaches the target but starts to oscillate (wiggle back and forth).
    //   2. Raise kD until the oscillation is dampened and the robot settles smoothly.
    //   3. Only touch kI if the robot consistently stops a few degrees short of the target.
    // =========================================================

    // TODO: kP — increase if the robot turns too slowly or undershoots the target.
    //            decrease if the robot oscillates (overshoots and wiggles) at the end.
    private static final double kP = 0.012;

    // TODO: kI — increase (very slightly) only if the robot always stops a few degrees short.
    //            keep at 0 until kP and kD are tuned. Too high = slow oscillation.
    private static final double kI = 0.003;

    // TODO: kD — increase if the robot oscillates after reaching the target.
    //            decrease if the robot slows down too early and creeps to the target.
    private static final double kD = 0.6;

    // TODO: TOLERANCE — how many degrees off is "close enough" to stop.
    //                   tighten (smaller) for precision, loosen (larger) for speed.
    private static final double TOLERANCE = 2.0;

    // TODO: MAX_POWER — clamps the maximum motor power during a turn.
    //                   lower if the robot overshoots consistently at full speed.
    private static final double MAX_POWER = 0.6;

    // MAX_INTEGRAL — caps how much the integral term can build up to prevent runaway power.
    // Only change if kI is non-zero and the robot surges unexpectedly.
    private static final double MAX_INTEGRAL = 30.0;

    private final Chassis   chassis;
    private final IMU       imu;
    private final Telemetry telemetry;

    // PID state — reset at the start of each new turn
    private double integral  = 0;
    private double prevError = 0;
    private double targetAngle = 0;
    private final ElapsedTime timer = new ElapsedTime();

    public ImuTurning(HardwareMap hardwareMap, Telemetry telemetry, Chassis chassis) {
        this.chassis   = chassis;
        this.telemetry = telemetry;

        imu = hardwareMap.get(IMU.class, "imu 1");
        // TODO: Adjust orientation to match your Control Hub mounting
        imu.initialize(new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
            )
        ));
    }

    /**
     * Call once to set the turn target before looping with update().
     * Resets PID state so each new turn starts clean.
     */
    public void startTurn(double targetAngleDeg) {
        this.targetAngle = targetAngleDeg;
        integral         = 0;
        prevError        = 0;
        timer.reset();
    }

    /**
     * Call every loop iteration after startTurn().
     *
     * @return true while still turning, false when within tolerance.
     */
    public boolean update() {
        double dt = timer.seconds();
        timer.reset();
        if (dt <= 0) dt = 0.02;

        double current = getHeading();
        double error   = angleWrap(targetAngle - current);

        if (Math.abs(error) < TOLERANCE) {
            chassis.stopChasis();
            integral  = 0;
            prevError = 0;
            return false;
        }

        integral += error * dt;
        integral  = Math.max(-MAX_INTEGRAL, Math.min(MAX_INTEGRAL, integral)); // anti-windup

        double derivative = (error - prevError) / dt;
        prevError = error;

        double power = kP * error + kI * integral + kD * derivative;
        power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));

        chassis.turn(power);

        telemetry.addData("IMU target",  "%.1f°", targetAngle);
        telemetry.addData("IMU heading", "%.1f°", current);
        telemetry.addData("IMU error",   "%.2f°", error);
        telemetry.addData("IMU power",   "%.3f",  power);

        return true;
    }

    public double getHeading() {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        return angles.getYaw(AngleUnit.DEGREES);
    }

    // Resets the IMU yaw to 0. Called by Odometry.resetPose() so both
    // classes always share the same heading reference point.
    public void resetYaw() {
        imu.resetYaw();
    }

    public double angleWrap(double angle) {
        while (angle >  180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
}
