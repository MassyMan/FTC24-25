package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TELEOP", group = "TeleOp")
public class Teleop extends OpMode {
    // Mecanum drive motors
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private DcMotor extendoEncoder;
    // CRServos for slide control
    private CRServo slidL, slidR;

    // Servo and CRServos for intake and v4Bar control
    private Servo v4Bar;
    private CRServo intake, intake2;

    // Vertical slide motors
    private DcMotor vertL, vertR;

    private static final int MAX_TICKS = 3800;
    private static final double HOLD_POWER = 0.15;

    private static final double MAX_EXTENDO = 16500;

    // V4Bar position limits
    private static final double V4BAR_MIN_POSITION = 0.24;
    private static final double V4BAR_MAX_POSITION = 0.8;
    private double v4BarPosition = 0.2; // V4Bar Assumption starting position (will travel to after being moved)
    private boolean v4BarMoved = false; // Flag to check if v4Bar has been moved

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

        extendoEncoder = hardwareMap.get(DcMotorEx.class, "leftBack");
        extendoEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendoEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize encoder and previous voltage
        // Analog encoder for slidL [extendo]
        AnalogInput axonR = hardwareMap.get(AnalogInput.class, "axonR");
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

        double currentExtendo = -extendoEncoder.getCurrentPosition();

        if (gamepad2.left_stick_y < 0) { // If Joystick is extending
            if (currentExtendo < MAX_EXTENDO) {
                slidL.setPower(-gamepad2.left_stick_y);
                slidR.setPower(gamepad2.left_stick_y);
            } else {
                slidL.setPower(0);
                slidR.setPower(0);
            }
        }

        if (gamepad2.left_stick_y > 0) {
            if (currentExtendo > 400) {
                slidL.setPower(-gamepad2.left_stick_y);
                slidR.setPower(gamepad2.left_stick_y);
            } else {
                slidL.setPower(0);
                slidR.setPower(0);
            }
        }

        if (gamepad2.left_stick_y == 0) {
            slidL.setPower(0);
            slidR.setPower(0);
        }

        if (gamepad2.y) {
            slidL.setPower(-0.1);
            slidR.setPower(0.1);
            extendoEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extendoEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Intake control
        if (gamepad2.left_bumper) {
            intake.setPower(-1.0); // Intake
            intake2.setPower(1.0); // Intake2 in the opposite direction
        } else if (gamepad2.left_trigger > 0.1) {
            intake.setPower(0.4); // Outtake
            intake2.setPower(-0.4); // Intake2 in the opposite direction
        } else {
            intake.setPower(0);
            intake2.setPower(0);
        }

        // v4Bar control (only moves after initial command)
        if (gamepad2.right_bumper) {
            v4BarPosition -= 0.025;
            v4BarMoved = true;
        } else if (gamepad2.right_trigger > 0.1) {
            v4BarPosition += 0.025;
            v4BarMoved = true;
        }

        if (gamepad2.dpad_left) {
            v4BarPosition = 0.43;
        }

        v4BarPosition = Range.clip(v4BarPosition, V4BAR_MIN_POSITION, V4BAR_MAX_POSITION);
        if (v4BarMoved) {
            v4Bar.setPosition(v4BarPosition);
        }

        if (gamepad2.dpad_up) {
            v4BarPosition = 0.27;
        }

        // Get the current position of the vertical slide motor (vertL)
        int currentPosition = vertL.getCurrentPosition();

// Vertical slide control with slow-down effect when lowering
        double powerScale;

        if (gamepad2.right_stick_y < 0) { // if raising slides
            if (currentPosition < MAX_TICKS) { // Move up until MAX_TICKS
                vertL.setPower(-gamepad2.right_stick_y); // Set motor power based on joystick input
                vertR.setPower(gamepad2.right_stick_y);
            } else {
                // Hold position at MAX_TICKS
                vertL.setPower(0);
                vertR.setPower(0);
            }
        } else if (gamepad2.right_stick_y > 0) { // if lowering slides
            if (currentPosition < 150) { // If near the bottom, apply no power
                vertL.setPower(0);
                vertR.setPower(0);
            } else {
                // Slow down effect when lowering
                // Gradual deceleration as the slides approach the bottom
                if (currentPosition > 600) {  // If the position is higher than the lower threshold
                    powerScale = 1.0; // Full speed initially
                } else {
                    powerScale = (currentPosition - 600) / 600.0; // Slow down as we approach the bottom
                    powerScale = Math.max(powerScale, 0.1); // Ensure it doesn't get too fast at the bottom
                }

                vertL.setPower(-gamepad2.right_stick_y * powerScale); // Apply scaled power based on position
                vertR.setPower(gamepad2.right_stick_y * powerScale);
            }
        } else if (gamepad2.right_stick_y == 0) { // Joystick not pressed (holding position)
            if (currentPosition > 800) {
                // Hold position at a reasonable threshold if the stick is released
                vertL.setPower(HOLD_POWER);
                vertR.setPower(-HOLD_POWER);
            } else if (currentPosition <= 150) {
                // Apply no power if near the bottom to save battery
                vertL.setPower(0);
                vertR.setPower(0);
            } else {
                // If it's too close to the bottom, apply a small power to prevent it from staying stuck
                vertL.setPower(0.1);
                vertR.setPower(-0.1);
            }
        }
        if (currentPosition < 0) {
            vertL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            vertL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }


        // Other controls for vertical slides and v4Bar
        if (gamepad2.dpad_right) {
            vertL.setPower(-0.06);
            vertR.setPower(0.06);
        }

        if (gamepad2.dpad_down) {
            if (currentPosition <= 1447) {
                vertL.setPower(1.0);
                vertR.setPower(-1.0);
            } else {
                vertL.setPower(HOLD_POWER);
                vertR.setPower(-HOLD_POWER);
            }
        }

        // Telemetry
        telemetry.addData("EXTENDO POSITION:", currentExtendo);
        telemetry.addLine();
        telemetry.addData("VERT SLIDE POSITION:", currentPosition);
        telemetry.update();
    }
}
