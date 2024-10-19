package org.firstinspires.ftc.teamcode;

// Necessary imports
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
    private double kP = 0.01; // Start with a small proportional value
    private double targetPosition = 0;
    private static final int THRESHOLD = 80; // Tolerance for holding position

    @Override
    public void runOpMode() {
        // Initialize motors
        vertL = hardwareMap.get(DcMotorEx.class, "vertL");
        vertR = hardwareMap.get(DcMotorEx.class, "vertR");

        // Set motor directions
        vertL.setDirection(DcMotorSimple.Direction.FORWARD);
        vertR.setDirection(DcMotorSimple.Direction.REVERSE); // Change direction for correct movement

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
            double joystickInput = -gamepad1.left_stick_y; // Invert input for expected behavior
            targetPosition = Range.clip(joystickInput * 3500, 0, 3500); // Scale joystick input to ticks

            // Get the current position of the left motor
            int currentPosition = vertL.getCurrentPosition();

            // Calculate the error
            double error = targetPosition - currentPosition;

            // Proportional control
            double power = kP * error;

            // Clip power to be within -1.0 to 1.0 range
            power = Range.clip(power, -1.0, 1.0);

            // Check if within threshold and hold position
            if (Math.abs(error) <= THRESHOLD) {
                vertL.setPower(0);  // Hold position
                vertR.setPower(0);
            } else {
                // Set power to both motors
                vertL.setPower(power);  // Positive power for raising
                vertR.setPower(-power); // Negative power for lowering
            }

            // Update telemetry
            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("Current Position", currentPosition);
            telemetry.addData("Power Output", power);
            telemetry.update();
        }

        // Stop motors after exiting the loop
        vertL.setPower(0);
        vertR.setPower(0);
    }
}
