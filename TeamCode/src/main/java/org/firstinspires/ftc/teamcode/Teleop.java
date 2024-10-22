package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "PIDF Teleop", group = "TeleOp")
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

    // PIDF Controller variables
    private double kP = 0.003;
    private double kF = 0.2;  // Reduce kF to prevent overshoot at the bottom
    private double targetPosition = 0;
    private static final int MAX_TICKS = 3900;
    private static final double MIN_DOWN_POWER = -0.2;
    private static final double ERROR_DEADBAND = 5;  // Error tolerance for stopping jitter

    // V4Bar position limits
    private static final double V4BAR_MIN_POSITION = 0.25;
    private static final double V4BAR_MAX_POSITION = 0.91;
    private double v4BarPosition = 0.19; // Start position for v4Bar

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

        // Initialize v4Bar position
        v4Bar.setPosition(v4BarPosition);
    }

    @Override
    public void loop() {
        // Gamepad 1: Mecanum drive control
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        // Slow mode with right trigger
        double speedMultiplier = gamepad1.right_trigger > 0.1 ? 0.3 : 1.0;

        double leftFrontPower = (drive + strafe + rotate) * speedMultiplier;
        double leftBackPower = (drive - strafe + rotate) * speedMultiplier;
        double rightFrontPower = (drive - strafe - rotate) * speedMultiplier;
        double rightBackPower = (drive + strafe - rotate) * speedMultiplier;

        leftFront.setPower(Range.clip(leftFrontPower, -1.0, 1.0));
        leftBack.setPower(Range.clip(leftBackPower, -1.0, 1.0));
        rightFront.setPower(Range.clip(rightFrontPower, -1.0, 1.0));
        rightBack.setPower(Range.clip(rightBackPower, -1.0, 1.0));

        // Gamepad 2: CRServos for slides and servo controls
        slidL.setPower(-gamepad2.left_stick_y);
        slidR.setPower(gamepad2.left_stick_y); // Opposite direction

        // Intake control
        if (gamepad2.left_bumper) {
            intake.setPower(1.0); // intake
        } else if (gamepad2.left_trigger > 0.1) {
            intake.setPower(-1.0); // outtake
        } else {
            intake.setPower(0); // Stop
        }

        // v4Bar position control
        if (gamepad2.right_bumper) {
            // Move v4Bar up (increase position)
            v4BarPosition -= 0.006; // Adjust increment as needed
        } else if (gamepad2.right_trigger > 0.1) {
            // Move v4Bar down (decrease position)
            v4BarPosition += 0.006; // Adjust decrement as needed
        }

        // Clip the v4Bar position to the defined limits
        v4BarPosition = Range.clip(v4BarPosition, V4BAR_MIN_POSITION, V4BAR_MAX_POSITION);
        v4Bar.setPosition(v4BarPosition);

        // Gamepad 2: Vertical slide control with PIDF (right joystick)
        double joystickInput = -gamepad2.right_stick_y;
        if (Math.abs(joystickInput) > 0.05) {
            targetPosition += joystickInput * 20;
            targetPosition = Range.clip(targetPosition, 0, MAX_TICKS);
        }

        int currentPosition = vertL.getCurrentPosition();
        double error = targetPosition - currentPosition;

        // Prevent twitching when both positions are at zero
        if (targetPosition == 0 && currentPosition <= 20) {
            vertL.setPower(0);
            vertR.setPower(0);
        } else if (Math.abs(error) < ERROR_DEADBAND) {
            vertL.setPower(0);
            vertR.setPower(0);
        } else {
            // Calculate motor power using PIDF
            double dynamicKF = (error < 0) ? 0.05 : kF; // Lower kF when moving down
            double power = kP * error + dynamicKF;
            power = Range.clip(power, MIN_DOWN_POWER, 1.0); // Limit power based on direction

            vertL.setPower(power);
            vertR.setPower(-power); // Reverse for opposite motor
        }

        // Telemetry for debugging
        telemetry.addData("Target Position", targetPosition);
        telemetry.addData("Current Position", currentPosition);
        telemetry.addData("Error", error);
        telemetry.addData("Power L", vertL.getPower());
        telemetry.addData("Power R", vertR.getPower());
        telemetry.update();
    }
}
