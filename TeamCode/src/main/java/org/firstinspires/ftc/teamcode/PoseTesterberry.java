package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "PoseTesterberry", group = "TeleOp")
public class PoseTesterberry extends OpMode {
    // Conversion factors and robot parameters
    private static final double IN_PER_TICK = 0.0019547157517511; // Parallel wheels
    private static final double LATERAL_IN_PER_TICK = 0.0019547157517511; // Perpendicular wheel
    private static final double TRACK_WIDTH_TICKS = 11.25; // Track width in
    private static final double PERP_OFFSET_TICKS = -2.75; // Perpendicular pod offset in

    // Robot pose
    private double x = 0.0;
    private double y = 0.0;
    private double theta = 0.0; // In radians

    // Previous encoder values
    private double lastPar0Ticks = 0.0;
    private double lastPar1Ticks = 0.0;
    private double lastPerpTicks = 0.0;

    // Odometry pods (should be DcMotorEx for better control)
    private DcMotorEx par0;
    private DcMotorEx par1;
    private DcMotorEx perp;

    @Override
    public void init() {
        // Initialize motors
        par0 = hardwareMap.get(DcMotorEx.class, "leftFront");
        par1 = hardwareMap.get(DcMotorEx.class, "rightFront");
        perp = hardwareMap.get(DcMotorEx.class, "rightBack");

        // Set the motors to run without encoders for odometry
        par0.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        par1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        perp.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Optionally, reset the motors' encoders to zero at the start of the program
        par0.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        par1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        perp.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        par0.setDirection(DcMotorSimple.Direction.FORWARD);
        par1.setDirection(DcMotorSimple.Direction.REVERSE);
        perp.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void loop() {
        // Update the robot's pose based on encoder data
        updatePose();

        // Convert theta from radians to degrees
        double thetaDegrees = Math.toDegrees(theta);

        // Optionally, display the pose on the telemetry
        telemetry.addData("X Position (in)", x);
        telemetry.addData("Y Position (in)", y);
        telemetry.addData("Heading (deg)", thetaDegrees); // Display theta in degrees
        telemetry.update();
    }

    public void updatePose() {
        // Get current encoder values
        double currentPar0Ticks = par0.getCurrentPosition();
        double currentPar1Ticks = par1.getCurrentPosition();
        double currentPerpTicks = perp.getCurrentPosition();

        // Calculate changes in encoder values (delta)
        double deltaPar0Ticks = currentPar0Ticks - lastPar0Ticks;
        double deltaPar1Ticks = currentPar1Ticks - lastPar1Ticks;
        double deltaPerpTicks = currentPerpTicks - lastPerpTicks;

        // Update previous values for the next iteration
        lastPar0Ticks = currentPar0Ticks;
        lastPar1Ticks = currentPar1Ticks;
        lastPerpTicks = currentPerpTicks;

        // Convert ticks to distances
        double deltaL = deltaPar0Ticks * IN_PER_TICK;
        double deltaR = deltaPar1Ticks * IN_PER_TICK;
        double deltaS = deltaPerpTicks * LATERAL_IN_PER_TICK;

        // Calculate change in heading (theta) using arc-based odometry
        double deltaTheta = (deltaR - deltaL) / TRACK_WIDTH_TICKS;

        // Correct strafe movement
        double deltaS_corrected = deltaS - deltaTheta * (PERP_OFFSET_TICKS * LATERAL_IN_PER_TICK);

        // Calculate the average distance traveled (arc-based)
        double deltaD = (deltaL + deltaR) / 2;

        // Update position based on arc movement
        // Update the robot's global position considering the robot's heading (theta)
        double deltaX = deltaD * Math.cos(theta + deltaTheta / 2) + deltaS_corrected * Math.sin(theta + deltaTheta / 2);
        double deltaY = deltaD * Math.sin(theta + deltaTheta / 2) - deltaS_corrected * Math.cos(theta + deltaTheta / 2);

        // Add the deltas to the current position
        x += deltaX;
        y += deltaY;
        theta += deltaTheta;

        // Normalize theta to [0, 360] degrees for display
        theta = normalizeAngle(theta);
    }

    // Normalize the angle to the range [0, 360] degrees
    private double normalizeAngle(double angle) {
        while (angle >= 2 * Math.PI) {
            angle -= 2 * Math.PI;
        }
        while (angle < 0) {
            angle += 2 * Math.PI;
        }
        return angle;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getTheta() {
        return theta;
    }
}
