package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TELEOP", group = "TeleOp")
public class Teleop extends OpMode {
    // Mecanum drive motors
    private DcMotor leftFront, leftBack, rightFront, rightBack;

    // CRServos for slide control
    private CRServo slidL, slidR;

    // Servo and CRServos for intake and v4Bar control
    private Servo v4Bar;
    private CRServo intake, intake2;

    // Vertical slide motors
    private DcMotor vertL, vertR;

    // Analog encoder for slidL [extendo]
    private AnalogInput axonR;

    private static final int MAX_TICKS = 3800;
    private static final double HOLD_POWER = 0.1;

    // V4Bar position limits
    private static final double V4BAR_MIN_POSITION = 0.25;
    private static final double V4BAR_MAX_POSITION = 0.928;
    private double v4BarPosition = 0.25; // V4Bar Assumption starting position (will travel to after being moved)
    private boolean v4BarMoved = false; // Flag to check if v4Bar has been moved

    // Slide rotation tracking [extendo]
    private double previousVoltage = 0;
    private int fullRotations = 0;

    // Voltage range constants for the encoder (analog signal) [extendo]
    private static final double VOLTAGE_MIN = 0;
    private static final double VOLTAGE_MAX = 3.4;
    private static final double VOLTAGE_RANGE = VOLTAGE_MAX - VOLTAGE_MIN;
    private static final double DEGREES_PER_VOLT = 360 / VOLTAGE_RANGE;
    private int rotationCount = 0;
    private int previousState = 0;

    // Threshold for detecting wraparound [extendo]
    private static final double WRAPAROUND_THRESHOLD = 1.65;

    // Limits for degrees [extendo]
    private static final double EXTEND_LIMIT_DEGREES = 800.0;
    private static final double RETRACT_LIMIT_DEGREES = 280.0;

    @Override
    public void init() {
        // Initialize motors for mecanum drive
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        // Set motor directions
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        // Initialize CRServos for slides
        slidL = hardwareMap.get(CRServo.class, "slidL");
        slidR = hardwareMap.get(CRServo.class, "slidR");

        // Initialize servos for v4Bar and intake
        v4Bar = hardwareMap.get(Servo.class, "v4Bar");
        intake = hardwareMap.get(CRServo.class, "intake");
        intake2 = hardwareMap.get(CRServo.class, "intake2");

        // Initialize vertical slide motors
        vertL = hardwareMap.get(DcMotor.class, "vertL");
        vertR = hardwareMap.get(DcMotor.class, "vertR");

        // Set vertical motors to brake at zero power
        vertL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vertR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize encoder and previous voltage
        axonR = hardwareMap.get(AnalogInput.class, "axonR");
        previousVoltage = axonR.getVoltage();
    }

    @Override
    public void loop() {
        // Mecanum drive control
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        double speedMultiplier = gamepad1.right_trigger > 0.1 ? 0.3 : 1.0;

        leftFront.setPower(Range.clip((drive + strafe + rotate) * speedMultiplier, -1.0, 1.0));
        leftBack.setPower(Range.clip((drive - strafe + rotate) * speedMultiplier, -1.0, 1.0));
        rightFront.setPower(Range.clip((drive - strafe - rotate) * speedMultiplier, -1.0, 1.0));
        rightBack.setPower(Range.clip((drive + strafe - rotate) * speedMultiplier, -1.0, 1.0));

        double currentVoltage = axonR.getVoltage();
        double currentDegrees = currentVoltage * DEGREES_PER_VOLT;
        double highRange = (((VOLTAGE_MAX - VOLTAGE_MIN) / 3) * 2); // Upper third range
        double lowRange = ((VOLTAGE_MAX - VOLTAGE_MIN) / 3);  // Lower third range
        int currentState;


// Determine the current state based on the voltage range
        if (currentVoltage >= highRange) {
            currentState = 3;
        } else if (currentVoltage <= lowRange) {
            currentState = 1;
        } else {
            currentState = 2;
        }

// Track the rotations
        if (previousState == 3 && currentState == 1) {
            rotationCount++; // Forward wrap
        } else if (previousState == 1 && currentState == 3) {
            rotationCount--; // Backward wrap
        }
        previousState = currentState;

        if (rotationCount == 2 && currentVoltage > 0.5){
            if (gamepad2.left_stick_y < 0.0) {
                slidL.setPower(0);
                slidR.setPower(0);
                telemetry.addData("EXTENDO AT EXTENSION LIMIT", "");
            } else {
                slidL.setPower(-gamepad2.left_stick_y);
                slidR.setPower(gamepad2.left_stick_y);
            }
        } else if (rotationCount == 0 && currentVoltage < 2.5){
                if (gamepad2.left_stick_y > 0.0) {
                    slidL.setPower(0);
                    slidR.setPower(0);
                    telemetry.addData("EXTENDO AT RETRACTION LIMIT", "");
                } else {
                    slidL.setPower(-gamepad2.left_stick_y);
                    slidR.setPower(gamepad2.left_stick_y);
                }
        } else {
            slidL.setPower(-gamepad2.left_stick_y);
            slidR.setPower(gamepad2.left_stick_y);
        }



// Update the previous state for the next loop
        previousState = currentState;


        // Slide rotation and extension control with reversed joystick direction
        /*
        double currentVoltage = axonR.getVoltage();
        double currentDegrees = currentVoltage * DEGREES_PER_VOLT;

        if (currentVoltage < VOLTAGE_MIN + WRAPAROUND_THRESHOLD && previousVoltage > VOLTAGE_MAX - WRAPAROUND_THRESHOLD) {
            fullRotations++;
        } else if (currentVoltage > VOLTAGE_MAX - WRAPAROUND_THRESHOLD && previousVoltage < VOLTAGE_MIN + WRAPAROUND_THRESHOLD) {
            fullRotations--;
        }

        double totalDegrees = (fullRotations * 360) + currentDegrees;

        // Reverse direction of left_stick_y for slide control
        double slidePower = -gamepad2.left_stick_y;
        double scaledPower = slidePower;

        // Check if close to extend or retract limits and adjust power accordingly
        if (slidePower > 0 && totalDegrees > (EXTEND_LIMIT_DEGREES - 100)) { // Approaching extend limit
            double error = EXTEND_LIMIT_DEGREES - totalDegrees;
            scaledPower = Range.clip(slidePower * (error / 100), 0.1, slidePower); // Slow down as error decreases
        } else if (slidePower < 0 && totalDegrees < (RETRACT_LIMIT_DEGREES + 100)) { // Approaching retract limit
            double error = totalDegrees - RETRACT_LIMIT_DEGREES;
            scaledPower = Range.clip(slidePower * (error / 100), slidePower, -0.1); // Slow down as error decreases
        }

        // Apply scaled power to the slides
        slidL.setPower(scaledPower);
        slidR.setPower(-scaledPower);

        previousVoltage = currentVoltage;


         */
        // Intake control
        if (gamepad2.left_bumper) {
            intake.setPower(-1.0); // Intake
            intake2.setPower(1.0); // Intake2 in the opposite direction
        } else if (gamepad2.left_trigger > 0.1) {
            intake.setPower(0.3); // Outtake
            intake2.setPower(-0.3); // Intake2 in the opposite direction
        } else {
            intake.setPower(0);
            intake2.setPower(0);
        }

        // v4Bar control (only moves after initial command)
        if (gamepad2.right_bumper) {
            v4BarPosition -= 0.005;
            v4BarMoved = true;
        } else if (gamepad2.right_trigger > 0.1) {
            v4BarPosition += 0.005;
            v4BarMoved = true;
        }

        if (gamepad2.dpad_left) {
            v4BarPosition = 0.5;
        }

        v4BarPosition = Range.clip(v4BarPosition, V4BAR_MIN_POSITION, V4BAR_MAX_POSITION);
        if (v4BarMoved) {
            v4Bar.setPosition(v4BarPosition);
        }

        if (gamepad2.dpad_up) {
            v4BarPosition = 0.33;
        }

// Get the current position of the vertical slide motor (vertL)
        int currentPosition = vertL.getCurrentPosition();

// Slide Control

        if (Math.abs(gamepad2.right_stick_y) > 0.1) {
            if (currentPosition < 100 && gamepad2.right_stick_y > 0.1) {
                // Prevent moving further down if already near the bottom
                vertL.setPower(0);
                vertR.setPower(0);
            } else if (currentPosition >= MAX_TICKS && gamepad2.right_stick_y < -0.1) {
                // Prevent moving further up if already at the maximum
                vertL.setPower(HOLD_POWER);
                vertR.setPower(-HOLD_POWER);
            } else if (currentPosition <= 900 && gamepad2.right_stick_y > 0.5) {
                // Slow down when slides are within 500 ticks from the bottom
                vertL.setPower(-0.5 * gamepad2.right_stick_y);
                vertR.setPower(0.5 * gamepad2.right_stick_y);
            } else {
                // Allow free movement up or down within the range
                vertL.setPower(-gamepad2.right_stick_y);
                vertR.setPower(gamepad2.right_stick_y);
            }
        } else if (currentPosition > 100 && currentPosition < MAX_TICKS) {
            // Hold position if no input and within the range
            vertL.setPower(HOLD_POWER);
            vertR.setPower(-HOLD_POWER);
        } else if (currentPosition >= MAX_TICKS && gamepad2.right_stick_y > 0.1) {
            // Allow moving down if at max position and joystick is pulling down
            vertL.setPower(-gamepad2.right_stick_y);
            vertR.setPower(gamepad2.right_stick_y);
        } else {
            // Stop the motors if no other condition applies
            vertL.setPower(0);
            vertR.setPower(0);
        }

        if (gamepad2.dpad_down){
            if (currentPosition <= 1580){
                vertL.setPower(1.0);
                vertR.setPower(-1.0);
            } else if (currentPosition >= 1580) {
                vertL.setPower(HOLD_POWER);
                vertR.setPower(-HOLD_POWER);
            }
        }

        // Telemetry
        telemetry.addData("EXTENDO Rotation Count:", rotationCount);
        telemetry.addData("EXTENDO Current State:", currentState);
        telemetry.addData("EXTENDO Previous State:", previousState);
        telemetry.addData("EXTENDO Current Voltage:", currentVoltage);
        telemetry.addData("GAMEPAD2.LEFT_STICK_Y:", gamepad2.left_stick_y);
        telemetry.addData("VERTS Current Position:", currentPosition);
        telemetry.addData("VERTS Target Stick", gamepad2.right_stick_y);
        telemetry.update();
    }
}
