package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp(name = "Slide Control with Encoder", group = "TeleOp")
public class SlideControlWithEncoder extends OpMode {
    // Define the servos and encoder
    private CRServo slidL, slidR;
    private AnalogInput axonR; // Analog encoder for slidL

    // Variables to track rotation
    private double previousVoltage = 0;
    private int fullRotations = 0;

    // Voltage range constants (assuming a 0-5V analog signal)
    private static final double VOLTAGE_MIN = 0.0;
    private static final double VOLTAGE_MAX = 3.3; // Adjust based on actual encoder range
    private static final double VOLTAGE_RANGE = VOLTAGE_MAX - VOLTAGE_MIN;
    private static final double DEGREES_PER_VOLT = 360 / VOLTAGE_RANGE;

    // Threshold for detecting wraparound between 360 and 0 degrees
    private static final double WRAPAROUND_THRESHOLD = 0.1; // Tolerance for crossing wraparound point

    // Limits for the slides
    private static final int MAX_ROTATIONS = 10; // Example: Adjust based on your slide limits

    @Override
    public void init() {
        // Initialize servos and encoder
        slidL = hardwareMap.get(CRServo.class, "slidL");
        slidR = hardwareMap.get(CRServo.class, "slidR");
        axonR = hardwareMap.get(AnalogInput.class, "axonR");

        previousVoltage = axonR.getVoltage();
    }

    @Override
    public void loop() {
        // Get the current encoder voltage and calculate the degrees
        double currentVoltage = axonR.getVoltage();
        double currentDegrees = currentVoltage * DEGREES_PER_VOLT;

        // Detect full rotations by monitoring voltage changes (crossing 0 or 360 degrees)
        if (currentVoltage < VOLTAGE_MIN + WRAPAROUND_THRESHOLD && previousVoltage > VOLTAGE_MAX - WRAPAROUND_THRESHOLD) {
            // Going from near 360° (high voltage) to 0° (low voltage), servo has completed a full rotation
            fullRotations++;
        } else if (currentVoltage > VOLTAGE_MAX - WRAPAROUND_THRESHOLD && previousVoltage < VOLTAGE_MIN + WRAPAROUND_THRESHOLD) {
            // Going from near 0° (low voltage) to 360° (high voltage), servo has completed a full reverse rotation
            fullRotations--;
        }

        // Calculate the total degrees traveled, including full rotations
        double totalDegrees = (fullRotations * 360) + currentDegrees;

        // Define the limits for extension and retraction
        if (gamepad2.left_stick_y > 0 && totalDegrees < MAX_ROTATIONS * 360) {
            // Extending slides: slidL positive, slidR negative
            slidL.setPower(1.0);
            slidR.setPower(-1.0);
        } else if (gamepad2.left_stick_y < 0 && totalDegrees > 0) {
            // Retracting slides: slidL negative, slidR positive
            slidL.setPower(-1.0);
            slidR.setPower(1.0);
        } else {
            // Stop servos if limits are reached
            slidL.setPower(0);
            slidR.setPower(0);
        }

        // Update previous voltage for next loop iteration
        previousVoltage = currentVoltage;

        // Telemetry for debugging
        telemetry.addData("Voltage", currentVoltage);
        telemetry.addData("Degrees", currentDegrees);
        telemetry.addData("Full Rotations", fullRotations);
        telemetry.addData("Total Degrees", totalDegrees);
        telemetry.update();
    }
}
