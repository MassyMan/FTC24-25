package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TELEOP", group = "TeleOp")
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
    private DcMotor wormL, wormR, wormEncoder;

    private static final double HOLD_POWER = 0.25;
    private static final double SPECIMEN_VERT = 695;
    private static final double VERT_MAX_TICKS = 1950;

    private static final double MIN_EXTENDO = 1500;
    private static final double MAX_EXTENDO = 15300;

    // V4Bar position limits
    private static final double V4BAR_MIN_POSITION = 0.17;
    private static final double V4BAR_MAX_POSITION = 0.72;

    private static final double OUTTAKE_SPEED = 0.27;
    private static final double INTAKE_SPEED = 1.0;

    private int HANGARMS_STATE = 2; // 1 = parallel to ground, 2 = vertical/safe, 3 = out, 4 = hanging

    private double v4BarPosition = 0.2; // V4Bar Assumption starting position (will travel to after being moved)
    private boolean v4BarMoved = false; // Flag to check if V4Bar has been moved

    private ElapsedTime ElapsedTime;

    private static final double HANGARMS_STATE_VERTICAL = 2700;
    private static final double HANGARMS_STATE_OUT = 7700;
    private static final double HANGARMS_STATE_HANGING1 = 600;
    private static final double HANGARMS_STATE_HANGING2 = 2200;
    private static final double HANGARMS_ENCODER_THRESHOLD = 250; // threshold for autonomous moving

    boolean wormManualControl = false;
    boolean hangArmsTriggered = false;
    boolean hangOneDone = false;
    boolean holdVertsIn = false;
    boolean holdExtendoIn = false;

    private double hangArmPos = 0;

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
        v4Bar = hardwareMap.get(Servo.class, "v4Bar");
        intakeL = hardwareMap.get(CRServo.class, "intakeL");
        intakeR = hardwareMap.get(CRServo.class, "intakeR");

        // Initialize vertical slide motors
        vertL = hardwareMap.get(DcMotor.class, "vertL");
        vertR = hardwareMap.get(DcMotor.class, "vertR");

        // Initialize worm drive motors
        wormL = hardwareMap.get(DcMotor.class, "wormL");
        wormR = hardwareMap.get(DcMotor.class, "wormR");
        wormEncoder = hardwareMap.get(DcMotor.class, "wormL");

        // Set vertical motors to brake at zero power
        vertL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vertR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        extendoEncoder = hardwareMap.get(DcMotorEx.class, "leftBack");
        extendoEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendoEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        wormL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wormL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wormR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wormR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Initialize encoder and previous voltage
        // Analog encoder for slidL [extendo]
    }

    @Override
    public void loop() {
        // Mecanum drive control

        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x * 1.3;
        double rotate = gamepad1.right_stick_x * 1.3;
        double speedMultiplier = gamepad1.right_trigger > 0.1 ? 0.3 : 1.0;

        leftFront.setPower(Range.clip((drive + strafe + rotate) * speedMultiplier, -1.0, 1.0));
        leftBack.setPower(Range.clip((drive - strafe + rotate) * speedMultiplier, -1.0, 1.0));
        rightFront.setPower(Range.clip((drive - strafe - rotate) * speedMultiplier, -1.0, 1.0));
        rightBack.setPower(Range.clip((drive + strafe - rotate) * speedMultiplier, -1.0, 1.0));

        double currentExtendo = -extendoEncoder.getCurrentPosition();
        if (!holdExtendoIn) {
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
        } else {
            slidL.setPower(0.1);
            slidR.setPower(-0.1);
            extendoEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extendoEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (gamepad2.y) {
            slidL.setPower(0.1);
            slidR.setPower(-0.1);
            extendoEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extendoEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Intake control
        if (gamepad2.left_bumper) {
            intakeL.setPower(INTAKE_SPEED); // Intake
            intakeR.setPower(-INTAKE_SPEED); // Intake2 in the opposite direction
        } else if (gamepad2.left_trigger > 0.1) {
            intakeL.setPower(-OUTTAKE_SPEED); // Outtake
            intakeR.setPower(OUTTAKE_SPEED); // Intake2 in the opposite direction
        } else {
            intakeL.setPower(0);
            intakeR.setPower(0);
        }
        if (currentExtendo <= 14000) {
            // V4Bar control (only moves after initial command)
            if (gamepad2.right_bumper) {
                v4BarPosition -= 0.025;
                v4BarMoved = true;
            } else if (gamepad2.right_trigger > 0) {
                v4BarPosition += 0.025;
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
        int currentVertPosition = vertL.getCurrentPosition();

        if (!holdVertsIn) {
// Vertical slide control with slow-down effect when lowering
            if (gamepad2.right_stick_y < 0) {
                if (currentVertPosition < VERT_MAX_TICKS) {
                    vertL.setPower(-gamepad2.right_stick_y);
                    vertR.setPower(gamepad2.right_stick_y);
                } else {
                    vertL.setPower(HOLD_POWER);
                    vertR.setPower(-HOLD_POWER);
                }
            } else if (gamepad2.right_stick_y > 0) {
                vertL.setPower(-gamepad2.right_stick_y);
                vertR.setPower(gamepad2.right_stick_y);
            } else if (gamepad2.right_stick_y == 0) {
                vertL.setPower(0);
                vertR.setPower(0);
            }
            if (currentVertPosition < 0) {
                vertL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                vertL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } else if (currentVertPosition > 400 && gamepad2.right_stick_y == 0) {
                vertL.setPower(HOLD_POWER);
                vertR.setPower(-HOLD_POWER);
            }


            // Other controls for vertical slides and V4Bar
            if (gamepad2.dpad_right) {
                vertL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                vertL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            if (gamepad2.dpad_down) {
                if (currentVertPosition <= SPECIMEN_VERT) {
                    vertL.setPower(1.0);
                    vertR.setPower(-1.0);
                } else {
                    vertL.setPower(HOLD_POWER);
                    vertR.setPower(-HOLD_POWER);
                }
            }
        } else {
            vertL.setPower(-0.15);
            vertR.setPower(0.15);
        }

        if (gamepad1.dpad_down) {
            wormManualControl = true;
            wormL.setPower(1.0);
            wormR.setPower(1.0);

        } else if (gamepad1.dpad_up) {
            wormManualControl = true;
            wormL.setPower(-1.0);
            wormR.setPower(-1.0);
        } else if (gamepad1.dpad_left){
            if (hangArmPos > 0) {
                wormL.setPower(-0.4);
                wormR.setPower(-0.4);
            } else {
                wormL.setPower(0);
                wormR.setPower(0);
            }
        } else if (wormManualControl){
            wormL.setPower(0);
            wormR.setPower(0);
        }

        if (gamepad1.left_stick_y > 0.1) {
            hangArmsTriggered = true;
        }


        hangArmPos = wormL.getCurrentPosition();
        if (gamepad1.b) {
            HANGARMS_STATE = 3;
            telemetry.addData("CHANGING STATE TO 2", "");
        }

        if (gamepad1.a && HANGARMS_STATE == 3) {
            HANGARMS_STATE = 4; // Hangarms state is set to hang, raising robot onto submersible
        }

            // LOGIC CONTROL FOR AUTONOMOUS HANG STATES
        if (!wormManualControl && hangArmsTriggered) {
            if (HANGARMS_STATE == 2) {
                if (hangArmPos < HANGARMS_STATE_VERTICAL - HANGARMS_ENCODER_THRESHOLD) {
                    wormL.setPower(0.5);
                    wormR.setPower(0.5);
                } else if (hangArmPos > HANGARMS_STATE_VERTICAL + HANGARMS_ENCODER_THRESHOLD) {
                    wormL.setPower(-0.5);
                    wormR.setPower(-0.5);
                } else {
                    wormL.setPower(0);
                    wormR.setPower(0);
                }
            } else if (HANGARMS_STATE == 3) {
                if (hangArmPos < HANGARMS_STATE_OUT - HANGARMS_ENCODER_THRESHOLD) {
                    wormL.setPower(0.8);
                    wormR.setPower(0.8);
                } else if (hangArmPos > HANGARMS_STATE_OUT + HANGARMS_ENCODER_THRESHOLD) {
                    wormL.setPower(-0.8);
                    wormR.setPower(-0.8);
                } else {
                    wormL.setPower(0);
                    wormR.setPower(0);
                }
            } else if (HANGARMS_STATE == 4 && !hangOneDone) {
                if (hangArmPos > HANGARMS_STATE_HANGING1) {
                    wormL.setPower(-1.0);
                    wormR.setPower(-1.0);

                    holdExtendoIn = true;
                    holdVertsIn = true;
                } else {
                    wormL.setPower(0);
                    wormR.setPower(0);

                    hangOneDone = true;
                    holdExtendoIn = true;
                    holdVertsIn = true;
                }
            } else if (HANGARMS_STATE == 4 && hangOneDone) {
                if (hangArmPos < HANGARMS_STATE_HANGING2) {
                    wormL.setPower(0.5);
                    wormR.setPower(0.5);
                    holdExtendoIn = true;
                    holdVertsIn = true;
                } else {
                    wormL.setPower(0);
                    wormR.setPower(0);
                    holdExtendoIn = false;
                    holdVertsIn = false;

                }
            }
        }



        // Telemetry
        telemetry.addData("EXTENDO POSITION:", currentExtendo);
        telemetry.addData("VERT SLIDE POSITION:", currentVertPosition);
        telemetry.addData("V4BAR POSITION:", v4Bar.getPosition());
        telemetry.addLine();
        telemetry.addData("VERTL POWER:", vertL.getPower());
        telemetry.addData("VERTR POWER:", vertR.getPower());
        telemetry.addLine();
        telemetry.addData("HANG POSITION:", hangArmPos);
        telemetry.addData("HANG STATE:", HANGARMS_STATE);
        telemetry.addData("WORM MANUAL CONTROL?:", wormManualControl);
        telemetry.update();
        telemetry.addData("LOOP TIME:", ElapsedTime.seconds());

        ElapsedTime.reset();
    }
}
