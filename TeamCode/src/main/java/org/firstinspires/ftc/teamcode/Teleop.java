package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Teleope", group = "TeleOp")
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

    // PIDF Controller variables
    private double kP = 0.002;
    private double kF = 0.5;
    private double targetPosition = 0;
    private static final int MAX_TICKS = 3900;
    private static final double MIN_DOWN_POWER = -0.1;
    private static final double ERROR_DEADBAND = 5;

    // V4Bar position limits
    private static final double V4BAR_MIN_POSITION = 0.25;
    private static final double V4BAR_MAX_POSITION = 0.968;
    private double v4BarPosition = 0.8;
    private boolean v4BarMoved = false; // Flag to check if v4Bar has been moved

    // Slide rotation tracking [extendo]
    private double previousVoltage = 0;
    private int fullRotations = 0;

    // Voltage range constants for the encoder (analog signal) [extendo]
    private static final double VOLTAGE_MIN = 0.0;
    private static final double VOLTAGE_MAX = 3.4;
    private static final double VOLTAGE_RANGE = VOLTAGE_MAX - VOLTAGE_MIN;
    private static final double DEGREES_PER_VOLT = 360 / VOLTAGE_RANGE;

    // Threshold for detecting wraparound [extendo]
    private static final double WRAPAROUND_THRESHOLD = 1.65;

    // Limits for degrees [extendo]
    private static final double EXTEND_LIMIT_DEGREES = 850.0;
    private static final double RETRACT_LIMIT_DEGREES = 255.0;

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

        // Slide rotation and extension control with reversed joystick direction
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

        // Intake control
        if (gamepad2.left_bumper) {
            intake.setPower(-1.0); // Intake
            intake2.setPower(1.0); // Intake2 in the opposite direction
        } else if (gamepad2.left_trigger > 0.1) {
            intake.setPower(1.0); // Outtake
            intake2.setPower(-1.0); // Intake2 in the opposite direction
        } else {
            intake.setPower(0);
            intake2.setPower(0);
        }

        // v4Bar control (only moves after initial command)
        if (gamepad2.right_bumper) {
            v4BarPosition -= 0.006;
            v4BarMoved = true;
        } else if (gamepad2.right_trigger > 0.1) {
            v4BarPosition += 0.006;
            v4BarMoved = true;
        }

        v4BarPosition = Range.clip(v4BarPosition, V4BAR_MIN_POSITION, V4BAR_MAX_POSITION);
        if (v4BarMoved) {
            v4Bar.setPosition(v4BarPosition);
        }

        // Vertical slide control
        double joystickInput = -gamepad2.right_stick_y;
        if (Math.abs(joystickInput) > 0.05) {
            targetPosition += joystickInput * 20;
            targetPosition = Range.clip(targetPosition, 0, MAX_TICKS);
        }

        // Get the current position of the vertical slide motor (vertL) and calculate error
        int currentPosition = vertL.getCurrentPosition();
        double error = targetPosition - currentPosition;

        if (targetPosition == 0 && currentPosition <= 15) {
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
        if (gamepad2.dpad_down) {
            targetPosition = 1600;
        }

        // Telemetry
        telemetry.addData("Degrees:", totalDegrees);
        telemetry.addData("Current Voltage:", currentVoltage);
        telemetry.addData("Vertical Target Position:", targetPosition);
        telemetry.addData("Vertical Current Position:", currentPosition);
        telemetry.update();
    }
}
