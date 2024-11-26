package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

    // Analog encoder for slidL [extendo]
    private AnalogInput axonR;

    private static final int MAX_TICKS = 3860;
    private static final double HOLD_POWER = 0.1;

    private static final double MAX_EXTENDO = 3800;

    // V4Bar position limits
    private static final double V4BAR_MIN_POSITION = 0.25;
    private static final double V4BAR_MAX_POSITION = 0.928;
    private double v4BarPosition = 0.25; // V4Bar Assumption starting position (will travel to after being moved)
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
        axonR = hardwareMap.get(AnalogInput.class, "axonR");


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


        slidL.setPower(-gamepad2.left_stick_y);
        slidR.setPower(gamepad2.left_stick_y);
           /* if (currentExtendo < MAX_EXTENDO && currentExtendo > 50) { // if within good travel range, move freely
                slidL.setPower(-gamepad2.left_stick_y);
                slidR.setPower(gamepad2.left_stick_y);
            } else if (currentExtendo > MAX_EXTENDO){
                if (gamepad2.left_stick_y > 0) { // If at limit, only allow retraction
                    slidL.setPower(-gamepad2.left_stick_y);
                    slidR.setPower(gamepad2.left_stick_y);
                } else {
                    slidL.setPower(0); // stop motors if trying to move beyond limit
                    slidR.setPower(0);
                }
            } else if (currentExtendo < 50) {
                if (gamepad2.left_stick_y < 0) { // If at limit, only allow extension
                    slidL.setPower(-gamepad2.left_stick_y);
                    slidR.setPower(gamepad2.left_stick_y);
                } else {
                    slidL.setPower(0); // stop motors if trying to retract beyond limit
                    slidR.setPower(0);
                }
            } */

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
            v4BarPosition -= 0.009;
            v4BarMoved = true;
        } else if (gamepad2.right_trigger > 0.1) {
            v4BarPosition += 0.009;
            v4BarMoved = true;
        }

        if (gamepad2.dpad_left) {
            v4BarPosition = 0.51;
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
        // Vertical slide control
        if (Math.abs(gamepad2.right_stick_y) > 0.01 &! gamepad2.dpad_right) {
            if (currentPosition < 50 && gamepad2.right_stick_y > 0.01) {
                vertL.setPower(0);
                vertR.setPower(0);
            } else if (currentPosition < 400 && gamepad2.right_stick_y > 0.01){
                vertL.setPower((-gamepad2.right_stick_y +0.5) / 6);
                vertR.setPower((gamepad2.right_stick_y +0.5) / 6);
            } else {
                vertL.setPower(-gamepad2.right_stick_y);
                vertR.setPower(gamepad2.right_stick_y);
            }

        } else if (gamepad2.dpad_right) {
            vertL.setPower(-0.07);
            vertR.setPower(0.07);
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
        telemetry.addData("GAMEPAD2.LEFT_STICK_Y:", gamepad2.left_stick_y);
        telemetry.addData("VERTS Current Position:", currentPosition);
        telemetry.addData("VERTS Target Stick", gamepad2.right_stick_y);
        telemetry.update();
    }
}
