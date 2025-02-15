package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Disabled
@TeleOp(name = "PID TO POINT", group = "TeleOp")
public class PoseTesterberry extends OpMode {
    // Conversion factors and robot parameters
    private static final double IN_PER_TICK = 0.0019547157517511; // Parallel wheels
    private static final double LATERAL_IN_PER_TICK = 0.0018547157517511; // Perpendicular wheel
    private static final double TRACK_WIDTH_IN = 11.25; // Track width in.
    private static final double PERP_OFFSET_IN = -2.75; // Perpendicular pod offset in.
    // Robot pose
    private double currentX = 0.0;
    private double currentY = 0.0;
    private double currentT = 0.0; // In radians
    // Previous encoder values
    private double lastPar0Ticks = 0.0;
    private double lastPar1Ticks = 0.0;
    private double lastPerpTicks = 0.0;
    // Odometry pods
    private DcMotorEx par0;
    private DcMotorEx par1;
    private DcMotorEx perp;
    // pose error
    private double errorX = 0;
    private double errorY = 0;
    private double errorT = 0;
    // previous error
    private double lastErrorX = 0;
    private double lastErrorY = 0;
    private double lastErrorT = 0;


    private double targetX = 0;
    private double targetY = 0;
    private double targetT = 0;

    private double xInput = 0;
    private double yInput = 0;
    private double tInput = 0;

    private double xKP = 0.1;
    private double xKD = 0.01;

    private double yKP = 0.;
    private double yKD = 0.;

    private double tKP = 0.;
    private double tKD = 0.;

    private ElapsedTime elapsedTime;


    @Override
    public void init() {
        // Initialize motors
        par0 = hardwareMap.get(DcMotorEx.class, "leftFront");
        par1 = hardwareMap.get(DcMotorEx.class, "rightFront");
        perp = hardwareMap.get(DcMotorEx.class, "rightBack");

        par0.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        par1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        perp.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        par0.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        par1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        perp.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        par0.setDirection(DcMotorSimple.Direction.FORWARD);
        par1.setDirection(DcMotorSimple.Direction.REVERSE);
        perp.setDirection(DcMotorSimple.Direction.FORWARD);

        elapsedTime = new ElapsedTime();
    }

    @Override
    public void loop() {
        // Update the robot's pose based on encoder data

        targetX = 10;
        targetY = 10;
        targetT = Math.toRadians(90);

        errorX = currentX - targetX;
        errorY = currentY - targetY;
        errorT = currentT - targetT;

        // pull current pose data each loop
        updatePose();
        calculatePIDs();

        telemetry.addData("X Position (in)", currentX);
        telemetry.addData("Y Position (in)", currentY);
        telemetry.addData("SDK Normalizer (heading in radians) [currentT]", currentT);
        telemetry.addData("Theta 360 scaling [normalizeAngle]", Math.toDegrees(currentT));
        telemetry.addLine();
        telemetry.addData("Target X:", targetX);
        telemetry.addData("Target Y:", targetY);
        telemetry.addData("Target T:", targetT);
        telemetry.addLine();
        telemetry.addData("errorX:", errorX);
        telemetry.addData("errorY:", errorY);
        telemetry.addData("errorT:", errorT);
        telemetry.addData("errorT [degrees]", Math.toDegrees(errorT));
        telemetry.addLine();
        telemetry.addData("Loop Time:", elapsedTime.time());
        telemetry.update();

        elapsedTime.reset();
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

        // Update previous values for the next iteration [usage is over for currents]
        lastPar0Ticks = currentPar0Ticks;
        lastPar1Ticks = currentPar1Ticks;
        lastPerpTicks = currentPerpTicks;

        lastErrorX = errorX;
        lastErrorY = errorY;
        lastErrorT = errorT;

        double deltaL = deltaPar0Ticks * IN_PER_TICK;
        double deltaR = deltaPar1Ticks * IN_PER_TICK;
        double deltaS = deltaPerpTicks * LATERAL_IN_PER_TICK;
        // Calculate change in heading (currentT) using arc-based odometry
        double deltaTheta = (deltaR - deltaL) / TRACK_WIDTH_IN;
        // Correct strafe movement
        double deltaS_corrected = deltaS - deltaTheta * (PERP_OFFSET_IN * LATERAL_IN_PER_TICK);
        // Calculate the average distance traveled (arc-based)
        double deltaD = (deltaL + deltaR) / 2;
        // Update position based on arc movement
        // Update the robot's global position considering the robot's heading (currentT)
        double deltaX = deltaD * Math.cos(currentT + deltaTheta / 2) + deltaS_corrected * Math.sin(currentT + deltaTheta / 2);
        double deltaY = deltaD * Math.sin(currentT + deltaTheta / 2) - deltaS_corrected * Math.cos(currentT + deltaTheta / 2);
        // Add the deltas to the current position
        currentX += deltaX;
        currentY += deltaY;
        currentT += deltaTheta;

        // Normalize currentT to [-180/180] thing;
        currentT = AngleUnit.normalizeRadians(currentT);

    }

    public void calculatePIDs(){
        xInput = (errorX * xKP) + (((errorX - lastErrorX) / elapsedTime.seconds()) * xKD);
        yInput = (errorY * yKP) + (((errorY - lastErrorY) / elapsedTime.seconds()) * yKD);
        tInput = (errorT * tKP) + (((errorT - lastErrorT) / elapsedTime.seconds()) * tKD);
    }

    // Normalize the angle to the range [0, 360] degrees
    // normalizeAngle(currentT)
    private double normalizeAngle(double angle) {
        while (angle >= 2 * Math.PI) {
            angle -= 2 * Math.PI;
        }
        while (angle < 0) {
            angle += 2 * Math.PI;
        }
        return angle;
    }
    public double getCurrentX() {
        return currentX;
    }
    public double getCurrentY() {
        return currentY;
    }
    public double getCurrentT() {
        return currentT;
    }
}