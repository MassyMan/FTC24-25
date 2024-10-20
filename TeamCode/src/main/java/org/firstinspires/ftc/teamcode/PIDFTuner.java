package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "PIDF Tuner Simplified", group = "TeleOp")
public class PIDFTuner extends LinearOpMode {
    private DcMotorEx vertL;
    private DcMotorEx vertR;

    // PIDF Controller variables
    private double kP = 0.002; // Proportional gain
    private double kF = 0.3;   // Feedforward gain
    private double targetPosition = 0;
    private static final int THRESHOLD = 80;
    private static final int MAX_TICKS = 3900; // Max encoder position for the slides
    private static final double HOLD_POWER = 0.1; // Small power to hold position
    private static final double POWER_ADJUSTMENT = 0.05; // Power adjustment when in threshold
    private static final double MIN_DOWN_POWER = -0.2; // Minimum power for downward movement

    @Override
    public void runOpMode() {
        // Initialize motors
        vertL = hardwareMap.get(DcMotorEx.class, "vertL");
        vertR = hardwareMap.get(DcMotorEx.class, "vertR");

        // Set motor directions
        vertL.setDirection(DcMotorSimple.Direction.FORWARD);
        vertR.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set zero power behavior to brake
        vertL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        vertR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Reset encoders
        vertL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        vertR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        vertL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        vertR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            // Set target position based on joystick input
            double joystickInput = -gamepad1.left_stick_y; // Joystick up should increase targetPosition
            if (Math.abs(joystickInput) > 0.05) { // Only adjust target position if joystick is significantly moved
                targetPosition += joystickInput * 20; // Increment target by joystick input (scaled)
                targetPosition = Range.clip(targetPosition, 0, MAX_TICKS); // Clip the target position within limits
            }

            // Get the current position of the left motor
            int currentPosition = vertL.getCurrentPosition();

            // Calculate the error
            double error = targetPosition - currentPosition;

            // Adjust kF dynamically based on the direction of movement
            double dynamicKF = (error < 0) ? 0.05 : kF; // Smaller feedforward when going down

            // Proportional control
            double power = kP * error + dynamicKF;

            // Clip power to be within -1.0 to 1.0 range
            power = Range.clip(power, -1.0, 1.0);

            // Check if within threshold and hold position
            boolean isInThreshold = Math.abs(error) <= THRESHOLD;
            if (isInThreshold) {
                // Apply a small hold power and a boost when close to the target
                power = HOLD_POWER + (error < 0 ? POWER_ADJUSTMENT : 0);
                vertL.setPower(power);
                vertR.setPower(-power);
            } else {
                // Set power to both motors
                if (power < 0) {
                    // Ensure a minimum power when moving down
                    power = Range.clip(power, MIN_DOWN_POWER, 1.0);
                }
                vertL.setPower(power);  // Positive power for raising
                vertR.setPower(-power); // Negative power for lowering
            }

            // Ensure the slides go fully to 0
            if (targetPosition == 0 && currentPosition <= 20) {
                vertL.setPower(0);
                vertR.setPower(0);
            }

            // Update telemetry
            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("Current Position", currentPosition);
            telemetry.addData("Power Output", power);
            telemetry.addData("In Threshold", isInThreshold ? "Yes" : "No"); // Display threshold status
            telemetry.update();
        }

        // Stop motors after exiting the loop
        vertL.setPower(0);
        vertR.setPower(0);
    }
}
