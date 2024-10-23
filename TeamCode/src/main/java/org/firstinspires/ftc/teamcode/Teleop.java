package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Integrated Teleop with Slide Control", group = "TeleOp")
public class Teleop extends OpMode {
    // Mecanum drive motors
    private DcMotor leftFront, leftBack, rightFront, rightBack;

    // CRServos for slide control
    private CRServo slidL, slidR;

    // Servo and CRServo for intake and v4Bar control
    private Servo v4Bar;
    private CRServo intake;

    // Vertical slide motors
    private DcMotor vertL, vertR;

    // Analog encoder for slidL
    private AnalogInput axonR;

    // PIDF Controller variables
    private double kP = 0.003;
    private double kF = 0.2;  // Reduce kF to prevent overshoot at the bottom
    private double targetPosition = 0;
    private static final int MAX_TICKS = 3900;
    private static final double MIN_DOWN_POWER = -0.2;
    private static final double ERROR_DEADBAND = 5;  // Error tolerance for stopping jitter

    // V4Bar position limits
    private static final double V4BAR_MIN_POSITION = 0.25;
    private static final double V4BAR_MAX_POSITION = 0.928;
    private double v4BarPosition = 0.19; // Start position for v4Bar

    // Slide rotation tracking
    private double previousVoltage = 0;
    private int fullRotations = 0;

    // Voltage range constants for the encoder (0-5V analog signal)
    private static final double VOLTAGE_MIN = 0.0;
    private static final double VOLTAGE_MAX = 3.39; // Adjust based on actual encoder range
    private static final double VOLTAGE_RANGE = VOLTAGE_MAX - VOLTAGE_MIN;
    private static final double DEGREES_PER_VOLT = 360 / VOLTAGE_RANGE;

    // Threshold for detecting wraparound
    private static final double WRAPAROUND_THRESHOLD = 1.5;

    // Limits for degrees
    private static final double EXTEND_LIMIT_DEGREES = 860.0;
    private static final double RETRACT_LIMIT_DEGREES = 160.0;

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

        // Initialize vertical slide motors
        vertL = hardwareMap.get(DcMotor.class, "vertL");
        vertR = hardwareMap.get(DcMotor.class, "vertR");

        // Set vertical motors to brake at zero power
        vertL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vertR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize encoder and previous voltage
        axonR = hardwareMap.get(AnalogInput.class, "axonR");
        previousVoltage = axonR.getVoltage();

        // Initialize v4Bar position
        v4Bar.setPosition(v4BarPosition);
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

        // Slide rotation and extension control with reversed joystick direction
        double currentVoltage = axonR.getVoltage();
        double currentDegrees = currentVoltage * DEGREES_PER_VOLT;

        if (gamepad2.dpad_down) {
            fullRotations = 0;
            previousVoltage = currentVoltage;
        }

        if (currentVoltage < VOLTAGE_MIN + WRAPAROUND_THRESHOLD && previousVoltage > VOLTAGE_MAX - WRAPAROUND_THRESHOLD) {
            fullRotations++;
        } else if (currentVoltage > VOLTAGE_MAX - WRAPAROUND_THRESHOLD && previousVoltage < VOLTAGE_MIN + WRAPAROUND_THRESHOLD) {
            fullRotations--;
        }

        double totalDegrees = (fullRotations * 360) + currentDegrees;

        // Reverse direction of left_stick_y for slide control
        if (gamepad2.left_stick_y < 0 && totalDegrees < EXTEND_LIMIT_DEGREES) {
            slidL.setPower(1.0);
            slidR.setPower(-1.0);
        } else if (gamepad2.left_stick_y > 0 && totalDegrees > RETRACT_LIMIT_DEGREES) {
            slidL.setPower(-1.0);
            slidR.setPower(1.0);
        } else {
            slidL.setPower(0);
            slidR.setPower(0);
        }

        previousVoltage = currentVoltage;

        // Intake control
        if (gamepad2.left_bumper) {
            intake.setPower(1.0); // Intake
        } else if (gamepad2.left_trigger > 0.1) {
            intake.setPower(-1.0); // Outtake
        } else {
            intake.setPower(0);
        }

        // v4Bar control
        if (gamepad2.right_bumper) {
            v4BarPosition -= 0.006;
        } else if (gamepad2.right_trigger > 0.1) {
            v4BarPosition += 0.006;
        }
        v4BarPosition = Range.clip(v4BarPosition, V4BAR_MIN_POSITION, V4BAR_MAX_POSITION);
        v4Bar.setPosition(v4BarPosition);

        // Vertical slide control
        double joystickInput = -gamepad2.right_stick_y;
        if (Math.abs(joystickInput) > 0.05) {
            targetPosition += joystickInput * 20;
            targetPosition = Range.clip(targetPosition, 0, MAX_TICKS);
        }

        int currentPosition = vertL.getCurrentPosition();
        double error = targetPosition - currentPosition;

        if (targetPosition == 0 && currentPosition <= 20) {
            vertL.setPower(0);
            vertR.setPower(0);
        } else if (Math.abs(error) < ERROR_DEADBAND) {
            vertL.setPower(0);
            vertR.setPower(0);
        } else {
            double dynamicKF = (error < 0) ? 0.05 : kF;
            double power = kP * error + dynamicKF;
            power = Range.clip(power, MIN_DOWN_POWER, 1.0);

            vertL.setPower(power);
            vertR.setPower(-power);
        }

        // Telemetry for debugging
        telemetry.addData("Target Position", targetPosition);
        telemetry.addData("Current Position", currentPosition);
        telemetry.addData("Error", error);
        telemetry.addData("Voltage", currentVoltage);
        telemetry.addData("Total Degrees", totalDegrees);
        telemetry.addData("Extend Limit", EXTEND_LIMIT_DEGREES);
        telemetry.addData("Retract Limit", RETRACT_LIMIT_DEGREES);
        telemetry.update();
    }
}
