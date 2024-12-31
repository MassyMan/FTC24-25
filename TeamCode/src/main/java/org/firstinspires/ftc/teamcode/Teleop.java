package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "NUTRON TELEOP", group = "TeleOp")
public class Teleop extends OpMode {
    // Mecanum drive motors
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private DcMotor extendoEncoder;
    // CRServos for slide control
    private CRServo slidL, slidR;

    // Servo and CRServos for intake and V4Bar control
    private Servo v4Bar;
    private CRServo intakeL, intakeR;

    // Vertical slide motors
    private DcMotor vertL, vertR;
    // worm drive motors
    private DcMotor wormL, wormR;

    private static final int MAX_TICKS = 2000;
    private static final double HOLD_POWER = 0.15;

    private static final double MIN_EXTENDO = 1000;
    private static final double MAX_EXTENDO = 15300;

    // V4Bar position limits
    private static final double V4BAR_MIN_POSITION = 0.17;
    private static final double V4BAR_MAX_POSITION = 0.72;

    private static final double OUTTAKE_SPEED = 0.27;
    private static final double INTAKE_SPEED = 1.0;

    private double v4BarPosition = 0.2; // V4Bar Assumption starting position (will travel to after being moved)
    private boolean v4BarMoved = false; // Flag to check if V4Bar has been moved

    private ElapsedTime ElapsedTime;

    @Override
    public void init() {

        ElapsedTime = new ElapsedTime();
        ElapsedTime.reset();
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

        // Initialize servos for V4Bar and intake
        v4Bar = hardwareMap.get(Servo.class, "V4Bar");
        intakeL = hardwareMap.get(CRServo.class, "intakeL");
        intakeR = hardwareMap.get(CRServo.class, "intakeR");

        // Initialize vertical slide motors
        vertL = hardwareMap.get(DcMotor.class, "vertL");
        vertR = hardwareMap.get(DcMotor.class, "vertR");

        // Initialize worm drive motors
        wormL = hardwareMap.get(DcMotor.class, "wormL");
        wormR = hardwareMap.get(DcMotor.class, "wormR");

        // Set vertical motors to brake at zero power
        vertL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vertR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        extendoEncoder = hardwareMap.get(DcMotorEx.class, "vertR");
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

        double currentExtendo = extendoEncoder.getCurrentPosition();

        if (gamepad2.left_stick_y < 0) { // If Joystick is extending
            if (currentExtendo < MAX_EXTENDO) {
                slidL.setPower(gamepad2.left_stick_y);
                slidR.setPower(-gamepad2.left_stick_y);
            } else {
                slidL.setPower(0);
                slidR.setPower(0);
            }
        }

        if (gamepad2.left_stick_y > 0) {
            if (currentExtendo > MIN_EXTENDO) {
                slidL.setPower(gamepad2.left_stick_y);
                slidR.setPower(-gamepad2.left_stick_y);
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
            slidL.setPower(0.1);
            slidR.setPower(-0.1);
            extendoEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extendoEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Intake control
        if (gamepad2.left_bumper) {
            intakeL.setPower(-INTAKE_SPEED); // Intake
            intakeR.setPower(INTAKE_SPEED); // Intake2 in the opposite direction
        } else if (gamepad2.left_trigger > 0.1) {
            intakeL.setPower(OUTTAKE_SPEED); // Outtake
            intakeR.setPower(-OUTTAKE_SPEED); // Intake2 in the opposite direction
        } else {
            intakeL.setPower(0);
            intakeR.setPower(0);
        }
    if (currentExtendo <= 14000) {
        // V4Bar control (only moves after initial command)
        if (gamepad2.right_bumper) {
            v4BarPosition -= 0.01;
            v4BarMoved = true;
        } else if (gamepad2.right_trigger > 0.1) {
            v4BarPosition += 0.01;
            v4BarMoved = true;
        }

        if (gamepad2.dpad_left) {
            v4BarPosition = 0.35;
        }

        v4BarPosition = Range.clip(v4BarPosition, V4BAR_MIN_POSITION, V4BAR_MAX_POSITION);
        if (v4BarMoved) {
            v4Bar.setPosition(v4BarPosition);
        }

        if (gamepad2.dpad_up) {
            v4BarPosition = 0.21;
        }
    }

        // Get the current position of the vertical slide motor (vertL)
        int currentPosition = vertL.getCurrentPosition();

// Vertical slide control with slow-down effect when lowering
        vertL.setPower(-gamepad2.right_stick_y); // Set motor power based on joystick input
        vertR.setPower(gamepad2.right_stick_y);

        if (currentPosition < 0) {
            vertL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            vertL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }


        // Other controls for vertical slides and V4Bar
        if (gamepad2.dpad_right) {
            vertL.setPower(-0.1);
            vertR.setPower(0.1);
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

        if (gamepad1.b) {
            wormL.setPower(1);
            wormR.setPower(1);
        } else if (gamepad1.a) {
            wormL.setPower(-1);
            wormR.setPower(-1);
        } else {
            wormL.setPower(0);
            wormR.setPower(0);
        }


        // Telemetry
        telemetry.addData("EXTENDO POSITION:", currentExtendo);
        telemetry.addData("VERT SLIDE POSITION:", currentPosition);
        telemetry.addData("V4BAR POSITION:", v4Bar.getPosition());
        telemetry.addData("LOOP TIME:", ElapsedTime.milliseconds() / 100);
        telemetry.update();

        ElapsedTime.reset();
    }
}
