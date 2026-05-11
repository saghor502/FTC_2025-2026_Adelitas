// =============================================================================
// FILE:    Odometry.java
// PACKAGE: org.firstinspires.ftc.teamcode.robot
// =============================================================================
// DESCRIPTION:
//   Tracks the robot's position (X, Y) and heading on the FTC field using
//   3 dead-wheel encoders and the IMU. Plans paths to target positions using
//   Breadth-First Search (BFS) on a 2"-resolution grid, then drives the robot
//   along that path using PID control.
//   If another robot shoves yours off course, it automatically replans.
//
// HOW DEAD WHEELS WORK:
//   Two parallel wheels (left + right) measure forward movement and help
//   calculate heading change. One perpendicular wheel (center) measures
//   sideways (strafe) movement. Together with the IMU, they give a full
//   pose: X, Y, and heading (θ).
//
// HARDWARE OWNERSHIP RULE:
//   - LEFT and RIGHT parallel encoder readings come from Chassis (getLF, getRF, getLB, getRB).
//     Chassis already owns those motor ports — this class never calls hardwareMap.get() for them.
//   - CENTER (strafe) encoder is the only port this class owns directly.
//   - Heading is read through ImuTurning — this class does NOT own the IMU.
//   - No OpMode should ever call hardwareMap.get() for any encoder or IMU port.
//
// HARDWARE REQUIRED:
//   Left + right encoder readings → via Chassis methods (no extra wiring)
//   Center dead wheel             → one encoder port on the Control/Expansion Hub (see TODO below)
//   ImuTurning instance           → heading source (no second IMU setup needed)
//   Chassis instance              → encoder reads + drive power
//
// USED BY:
//   - OdometryNavigationExample.java  (usage reference)
//   - Any autonomous OpMode that needs position tracking
//
// HOW TO USE:
//   1. Create ImuTurning first, then pass it to Odometry in OpMode.init():
//        imuTurning = new ImuTurning(hardwareMap, telemetry, chassis);
//        odometry   = new Odometry(hardwareMap, telemetry, chassis, imuTurning);
//   2. Set starting pose (where the robot physically is on the field):
//        odometry.resetPose(72, 72, 0);  // X inches, Y inches, heading degrees
//   3. Plan a path to a target:
//        odometry.planPath(96, 72, 0);   // targetX, targetY, targetHeading
//   4. Execute the path every loop (returns false when arrived):
//        while (opModeIsActive() && odometry.navigatePath()) {
//            odometry.update();
//            telemetry.update();
//        }
//   IMPORTANT: always call odometry.update() every loop, even outside navigation.
//
// TUNING — see TODO comments on the PID constants and physical measurements below.
// =============================================================================
package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;

public class Odometry {

    // =========================================================
    // HARDWARE NAMES
    // =========================================================
    // Left and right parallel encoders are read through Chassis — no name needed here.
    // TODO: Set this to the hardware map name of the port where the CENTER (strafe) dead wheel
    //       encoder is plugged in. This must be a port NOT used by Chassis.
    //       Example: if you plug it into an expansion hub slot named "lb", set:
    //       CENTER_ENCODER_NAME = "lb"
    private static final String CENTER_ENCODER_NAME = "lb"; // TODO

    // TODO: Choose which Chassis encoder ports your LEFT and RIGHT parallel dead wheels use.
    //       Change the getLeft() and getRight() helper methods below to match.
    //       Options from Chassis: getLF(), getLB() for left side — getRF(), getRB() for right side.

    // =========================================================
    // ROBOT PHYSICAL MEASUREMENTS — measure from your actual robot
    // =========================================================
    // TODO: Check your dead wheel encoder datasheet for ticks per revolution
    //       Common values: REV Through Bore Encoder = 8192, goBILDA Odometry = 2000
    private static final double TICKS_PER_REV = 2000.0;

    // TODO: Measure the diameter of your dead wheels in inches
    private static final double WHEEL_DIAMETER_INCHES = 1.5;
    private static final double TICKS_PER_INCH = TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_INCHES);

    // TODO: Measure the distance between the LEFT and RIGHT parallel dead wheels (in inches)
    //       This is the lateral separation between the two parallel wheels.
    private static final double TRACK_WIDTH_INCHES = 10.0;

    // TODO: Measure the perpendicular distance from the robot center to the CENTER (strafe) dead wheel (in inches)
    //       Positive if the strafe wheel is in front of the robot center, negative if behind.
    private static final double LATERAL_OFFSET_INCHES = 3.0;

    // =========================================================
    // FIELD GRID — FTC field is 144" x 144"
    // Origin (0, 0) = robot starting corner. Grid covers the full field.
    // =========================================================
    private static final double GRID_RESOLUTION = 2.0; // inches per BFS cell
    private static final int    GRID_SIZE        = 72;  // 144" / 2" = 72 cells per axis

    // =========================================================
    // PID CONSTANTS — TRANSLATION (X/Y movement toward each waypoint)
    // Tuning order: set kI_XY = 0, kD_XY = 0 first.
    //   1. Raise kP_XY until the robot moves toward waypoints but starts to oscillate.
    //   2. Raise kD_XY to dampen the oscillation.
    //   3. Add kI_XY only if the robot consistently stops short of the waypoint.
    // =========================================================

    // TODO: kP_XY — increase if the robot moves too slowly toward each waypoint.
    //               decrease if the robot oscillates or overshoots the waypoint.
    private static final double kP_XY = 0.05;

    // TODO: kI_XY — increase (very slightly) only if the robot always stops inches short of the waypoint.
    //               keep at 0 until kP_XY and kD_XY are tuned. Too high = slow oscillation.
    private static final double kI_XY = 0.001;

    // TODO: kD_XY — increase if the robot oscillates as it approaches the waypoint.
    //               decrease if the robot slows down too early and creeps to the target.
    private static final double kD_XY = 0.005;

    // =========================================================
    // PID CONSTANTS — HEADING (rotation to final target angle)
    // Same tuning order as translation: kP first, then kD, kI last.
    // =========================================================

    // TODO: kP_HEADING — increase if the robot rotates too slowly to the target angle.
    //                    decrease if it overshoots and spins back and forth.
    private static final double kP_HEADING = 0.8;

    // TODO: kI_HEADING — increase (very slightly) only if the robot always stops a few degrees short.
    //                    keep at 0 until kP_HEADING and kD_HEADING are tuned.
    private static final double kI_HEADING = 0.0;

    // TODO: kD_HEADING — increase if the robot oscillates (wiggles) when settling on the target angle.
    //                    decrease if it slows down too early and barely reaches the target.
    private static final double kD_HEADING = 0.05;

    // TODO: XY_TOLERANCE_INCHES — how many inches from a waypoint counts as "arrived".
    //                             smaller = more precise path, larger = faster but sloppier.
    private static final double XY_TOLERANCE_INCHES = 1.0;

    // TODO: HEADING_TOLERANCE_DEG — how many degrees off counts as "aligned".
    //                               smaller = more precise heading, larger = faster.
    private static final double HEADING_TOLERANCE_DEG = 2.0;

    // TODO: MAX_TRANSLATE_POWER — clamps top speed during waypoint following.
    //                             lower if the robot overshoots waypoints at full speed.
    private static final double MAX_TRANSLATE_POWER = 0.7;

    // TODO: MAX_ROTATE_POWER — clamps top speed during final heading alignment.
    //                          lower if the robot overshoots the target angle.
    private static final double MAX_ROTATE_POWER = 0.4;

    // TODO: REPLAN_THRESHOLD_INCHES — how far the robot must be shoved off the current waypoint
    //                                 before BFS replans the path from scratch.
    //                                 lower = more reactive to pushes, higher = less replanning noise.
    private static final double REPLAN_THRESHOLD_INCHES = 6.0;

    // 8-directional BFS move offsets (mecanum can move in any direction)
    private static final int[][] DIRS = {
        { 1,  0}, {-1,  0}, { 0,  1}, { 0, -1},
        { 1,  1}, { 1, -1}, {-1,  1}, {-1, -1}
    };

    // =========================================================
    // HARDWARE
    // =========================================================
    // Left + right encoders are read through Chassis — it already owns those motor ports.
    private final DcMotor    centerEncoder; // only encoder this class owns directly
    private final ImuTurning imuTurning;   // shared IMU — configured once in ImuTurning
    private final Chassis    chassis;
    private final Telemetry  telemetry;

    // =========================================================
    // POSE STATE  (inches, inches, radians)
    // =========================================================
    private double x, y, headingRad;
    private int prevLeft, prevRight, prevCenter; // previous encoder tick counts

    // =========================================================
    // PID STATE
    // =========================================================
    private double integralX, integralY, prevErrorX, prevErrorY;
    private double integralH, prevErrorH;
    private final ElapsedTime pidTimer = new ElapsedTime();

    // =========================================================
    // PATH STATE
    // =========================================================
    private List<int[]> path      = new ArrayList<>();
    private int         pathIndex = 0;
    private boolean     navigating = false;
    private double      targetHeadingDeg = 0;

    // =========================================================
    // CONSTRUCTOR
    // =========================================================
    // imuTurning must be created before Odometry and passed in here.
    // This way both classes share the same IMU — no duplicate initialization.
    public Odometry(HardwareMap hardwareMap, Telemetry telemetry, Chassis chassis, ImuTurning imuTurning) {
        this.chassis    = chassis;
        this.telemetry  = telemetry;
        this.imuTurning = imuTurning;

        // Left + right encoders are read through Chassis — no hardwareMap.get() needed here.
        // Center (strafe) dead wheel is the only encoder this class owns directly.
        centerEncoder = hardwareMap.get(DcMotor.class, CENTER_ENCODER_NAME);
        centerEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        centerEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        resetPose(0, 0, 0);
    }

    // =========================================================
    // POSE TRACKING  — call update() every loop iteration
    // =========================================================

    /**
     * Updates X, Y, and heading from dead wheels + IMU.
     * Must be called every loop iteration (TeleOp and Autonomous).
     */
    public void update() {
        int leftNow   = getLeft();
        int rightNow  = getRight();
        int centerNow = centerEncoder.getCurrentPosition();

        double dLeft   = (leftNow   - prevLeft)   / TICKS_PER_INCH;
        double dRight  = (rightNow  - prevRight)  / TICKS_PER_INCH;
        double dCenter = (centerNow - prevCenter) / TICKS_PER_INCH;

        prevLeft   = leftNow;
        prevRight  = rightNow;
        prevCenter = centerNow;

        // IMU is primary heading source; read through ImuTurning (single shared instance)
        double newHeading = Math.toRadians(imuTurning.getHeading());
        double dTheta     = angleWrapRad(newHeading - headingRad);
        double midHeading = headingRad + dTheta / 2.0; // midpoint angle for arc integration

        headingRad = newHeading;

        double dForward = (dLeft + dRight) / 2.0;
        double dStrafe  = dCenter - (LATERAL_OFFSET_INCHES * dTheta); // remove heading-induced strafe

        // Rotate robot-local deltas into global field frame
        x += dForward * Math.cos(midHeading) - dStrafe * Math.sin(midHeading);
        y += dForward * Math.sin(midHeading) + dStrafe * Math.cos(midHeading);

        telemetry.addData("Pose", "(%.2f\", %.2f\") %.1f°", x, y, Math.toDegrees(headingRad));
    }

    /**
     * Resets the stored pose. Call at the start of autonomous with the robot's known field position.
     *
     * @param startX          starting X in inches
     * @param startY          starting Y in inches
     * @param startHeadingDeg starting heading in degrees
     */
    public void resetPose(double startX, double startY, double startHeadingDeg) {
        x          = startX;
        y          = startY;
        headingRad = Math.toRadians(startHeadingDeg);
        prevLeft   = getLeft();
        prevRight  = getRight();
        prevCenter = centerEncoder.getCurrentPosition();
        resetPidState();
        imuTurning.resetYaw();
        pidTimer.reset();
    }

    public double getX()          { return x; }
    public double getY()          { return y; }
    public double getHeadingDeg() { return Math.toDegrees(headingRad); }

    // =========================================================
    // PATHFINDING — BFS + PID navigation
    // =========================================================

    /**
     * Plans a BFS path from the current pose to the target position.
     * Then call navigatePath() every loop to execute it.
     *
     * @param targetX          destination X in inches
     * @param targetY          destination Y in inches
     * @param targetHeadingDeg heading the robot should face upon arrival
     */
    public void planPath(double targetX, double targetY, double targetHeadingDeg) {
        this.targetHeadingDeg = targetHeadingDeg;

        int startCol = worldToGrid(x);
        int startRow = worldToGrid(y);
        int goalCol  = worldToGrid(targetX);
        int goalRow  = worldToGrid(targetY);

        path       = bfs(startCol, startRow, goalCol, goalRow);
        pathIndex  = 0;
        navigating = !path.isEmpty();
        resetPidState();
        pidTimer.reset();
    }

    /**
     * Executes the current path one step per loop iteration.
     * If the robot is shoved off course, replans automatically.
     *
     * @return true while still navigating, false when the destination and heading are reached.
     */
    public boolean navigatePath() {
        if (!navigating || path.isEmpty()) return false;

        if (pathIndex < path.size()) {
            int[]  cell = path.get(pathIndex);
            double wpX  = gridToWorld(cell[0]);
            double wpY  = gridToWorld(cell[1]);
            double dist = Math.hypot(wpX - x, wpY - y);

            // Replan if another robot shoved us far off the current waypoint
            if (dist > REPLAN_THRESHOLD_INCHES) {
                int[] goal = path.get(path.size() - 1);
                planPath(gridToWorld(goal[0]), gridToWorld(goal[1]), targetHeadingDeg);
                return true;
            }

            if (dist < XY_TOLERANCE_INCHES) {
                pathIndex++;
                resetPidState();
                pidTimer.reset();
            }
        }

        if (pathIndex >= path.size()) {
            // All waypoints reached — align final heading
            if (alignHeading(targetHeadingDeg)) {
                chassis.stopChasis();
                navigating = false;
                return false;
            }
            return true;
        }

        int[]  cell = path.get(pathIndex);
        driveToPoint(gridToWorld(cell[0]), gridToWorld(cell[1]));
        return true;
    }

    public boolean isNavigating() { return navigating; }

    // =========================================================
    // PRIVATE — BFS
    // =========================================================

    private List<int[]> bfs(int startCol, int startRow, int goalCol, int goalRow) {
        List<int[]> result = new ArrayList<>();
        if (!inBounds(startCol, startRow) || !inBounds(goalCol, goalRow)) return result;

        int total    = GRID_SIZE * GRID_SIZE;
        int[][] prev = new int[total][2];
        boolean[] visited = new boolean[total];
        for (int[] p : prev) { p[0] = -1; p[1] = -1; }

        ArrayDeque<int[]> queue = new ArrayDeque<>();
        queue.add(new int[]{startCol, startRow});
        visited[idx(startCol, startRow)] = true;

        boolean found = false;
        while (!queue.isEmpty()) {
            int[] cur = queue.poll();
            if (cur[0] == goalCol && cur[1] == goalRow) { found = true; break; }
            for (int[] d : DIRS) {
                int nc = cur[0] + d[0];
                int nr = cur[1] + d[1];
                if (inBounds(nc, nr) && !visited[idx(nc, nr)]) {
                    visited[idx(nc, nr)] = true;
                    prev[idx(nc, nr)]    = cur.clone();
                    queue.add(new int[]{nc, nr});
                }
            }
        }

        if (!found) return result;

        // Reconstruct path from goal back to start, skip the start cell itself
        int[] cur = new int[]{goalCol, goalRow};
        while (cur[0] != startCol || cur[1] != startRow) {
            result.add(0, cur);
            cur = prev[idx(cur[0], cur[1])];
        }
        return result;
    }

    // =========================================================
    // PRIVATE — PID drive
    // =========================================================

    private void driveToPoint(double targetX, double targetY) {
        double dt = pidTimer.seconds();
        pidTimer.reset();
        if (dt <= 0) dt = 0.02;

        double errX = targetX - x;
        double errY = targetY - y;

        integralX += errX * dt;
        integralY += errY * dt;

        double forward = kP_XY * errX + kI_XY * integralX + kD_XY * ((errX - prevErrorX) / dt);
        double strafe  = kP_XY * errY + kI_XY * integralY + kD_XY * ((errY - prevErrorY) / dt);

        prevErrorX = errX;
        prevErrorY = errY;

        // Rotate global drive vector into robot-local frame
        double h = headingRad;
        double localForward = forward * Math.cos(h) + strafe * Math.sin(h);
        double localStrafe  = -forward * Math.sin(h) + strafe * Math.cos(h);

        chassis.mecanumDrive(
            clamp(localForward, MAX_TRANSLATE_POWER),
            clamp(localStrafe,  MAX_TRANSLATE_POWER),
            0,
            false
        );

        telemetry.addData("WP target", "(%.1f\", %.1f\")", targetX, targetY);
        telemetry.addData("WP error",  "(%.2f\", %.2f\")", errX, errY);
    }

    /** @return true when heading is within tolerance */
    private boolean alignHeading(double targetDeg) {
        double dt = pidTimer.seconds();
        pidTimer.reset();
        if (dt <= 0) dt = 0.02;

        double errH = angleWrapDeg(targetDeg - Math.toDegrees(headingRad));
        if (Math.abs(errH) < HEADING_TOLERANCE_DEG) return true;

        integralH += errH * dt;
        double rotate = kP_HEADING * errH + kI_HEADING * integralH + kD_HEADING * ((errH - prevErrorH) / dt);
        prevErrorH = errH;

        chassis.mecanumDrive(0, 0, clamp(rotate, MAX_ROTATE_POWER), false);
        return false;
    }

    // =========================================================
    // PRIVATE — helpers
    // =========================================================

    private void resetPidState() {
        integralX = integralY = prevErrorX = prevErrorY = 0;
        integralH = prevErrorH = 0;
    }

    // TODO: Change getLF() / getRF() to getLB() / getRB() if your parallel dead wheels
    //       are plugged into the back motor encoder ports instead of the front ones.
    private int getLeft()  { return chassis.getLF(); }
    private int getRight() { return chassis.getRF(); }

    private int    worldToGrid(double inches) { return (int) Math.round(inches / GRID_RESOLUTION); }
    private double gridToWorld(int cell)      { return cell * GRID_RESOLUTION; }
    private boolean inBounds(int c, int r)   { return c >= 0 && c < GRID_SIZE && r >= 0 && r < GRID_SIZE; }
    private int    idx(int c, int r)          { return r * GRID_SIZE + c; }

    private double angleWrapRad(double rad) {
        while (rad >  Math.PI) rad -= 2 * Math.PI;
        while (rad < -Math.PI) rad += 2 * Math.PI;
        return rad;
    }

    private double angleWrapDeg(double deg) {
        while (deg >  180) deg -= 360;
        while (deg < -180) deg += 360;
        return deg;
    }

    private double clamp(double val, double max) {
        return Math.max(-max, Math.min(max, val));
    }
}
