package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "[LEFT] Bucket 4+0", group = "Autonomous")
public class LeftSideAutoPIDF extends LinearOpMode {

    private SlideLift slideLift;
    private Servo v4Bar;
    private CRServo intake;
    private CRServo intake2;

    // PIDF control variables
    public static double kP = 0.005;
    public static double kF = 0.2;
    public static final int THRESHOLD = 80;
    private static final int MAX_TICKS = 3900;
    private static final double HOLD_POWER = 0.1;
    private static final double MIN_DOWN_POWER = -0.90;

    public class SlideLift {
        private DcMotorEx vertL;
        private DcMotorEx vertR;
        private double targetPosition = 0;

        public SlideLift(HardwareMap hardwareMap) {
            vertL = hardwareMap.get(DcMotorEx.class, "vertL");
            vertR = hardwareMap.get(DcMotorEx.class, "vertR");

            vertL.setDirection(DcMotorSimple.Direction.FORWARD);
            vertR.setDirection(DcMotorSimple.Direction.REVERSE);

            vertL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            vertR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

            vertL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            vertR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            vertL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            vertR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        public void moveSlides(double targetTicks) {
            targetPosition = Range.clip(targetTicks, 0, MAX_TICKS);
            int currentPosition = vertL.getCurrentPosition();
            double error = targetPosition - currentPosition;
            double power = kP * error + kF;
            power = Range.clip(power, MIN_DOWN_POWER, 1.0);

            if (power < 0) {
                power = Math.max(power, MIN_DOWN_POWER);
            }



            // If the target position is zero and slides are below 100 ticks, stop the motors
            if ((currentPosition <= 150) && (power < 0) && targetPosition <= 100) {
                vertL.setPower(0);
                vertR.setPower(0);
                telemetry.addData("Slide Lift", "Stopping power, gravity pulling to 0");
                telemetry.update();
                return;
            }

            // Set motor power
            vertL.setPower(power);
            vertR.setPower(power);

            // Check if within the threshold for holding power at non-zero targets
            if (Math.abs(error) <= THRESHOLD + 50 && targetPosition != 0) {
                vertL.setPower(HOLD_POWER);
                vertR.setPower(HOLD_POWER);
                telemetry.addData("Slide Lift", "Holding at Position: %d", currentPosition);
                telemetry.update();
                return;
            }





            telemetry.addData("Slide Lift", "Target Position: %d", (int) targetPosition);
            telemetry.addData("Current Position", currentPosition);
            telemetry.addData("Error", error);
            telemetry.addData("Power Output", power);
            telemetry.addData("Is At Target", isAtTarget());
            telemetry.update();
        }


        public boolean isAtTarget() {
            return Math.abs(targetPosition - vertL.getCurrentPosition()) <= THRESHOLD + 30; // +10 to fix back out glitch
        }
    }

    public class SlideLiftAction implements Action {
        private SlideLift slideLift;
        private double targetTicks;

        public SlideLiftAction(SlideLift slideLift, double targetTicks) {
            this.slideLift = slideLift;
            this.targetTicks = Range.clip(targetTicks, 0, MAX_TICKS);
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            slideLift.moveSlides(targetTicks);
            boolean isAtTarget = slideLift.isAtTarget();
            telemetry.addData("SlideLiftAction", "At Target: %b, Target Ticks: %.2f", isAtTarget, targetTicks);
            telemetry.update();
            return !isAtTarget;
        }
    }

    public class V4BarAction implements Action {
        private Servo v4Bar;
        private double position;

        public V4BarAction(Servo v4Bar, double position) {
            this.v4Bar = v4Bar;
            this.position = position;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            v4Bar.setPosition(position);
            telemetry.addData("V4Bar", "Moving to Position: %.2f", position);
            telemetry.update();
            return false;
        }
    }

    public class IntakeSpinAction implements Action {
        private CRServo intake;
        private CRServo intake2;
        private double power;
        private double duration;
        private ElapsedTime timer;
        private boolean timerStarted = false; // Flag to track if the timer has started

        public IntakeSpinAction(CRServo intake, CRServo intake2, double power, double duration) {
            this.intake = intake;
            this.intake2 = intake2;
            this.power = power;
            this.duration = duration;
            this.timer = new ElapsedTime();
            timerStarted = false;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            // Only reset the timer once, at the start of the action
            if (!timerStarted) {
                timer.reset();
                timerStarted = true;
            }

            // Run the intake while the elapsed time is less than the specified duration
            if (timer.seconds() < duration) {
                intake.setPower(power);
                intake2.setPower(-power);
                telemetry.addData("Intake", "Running at power: %.2f for %.2f seconds", power, duration);
                telemetry.update();
                return true;
            } else {
                // Stop the intake and mark the action as complete
                intake.setPower(0);
                intake2.setPower(0);
                telemetry.addData("Intake", "Stopped after duration: %.2f seconds", duration);
                telemetry.update();
                timerStarted = false;
                timer.reset();
                return false;
            }
        }
    }



    @Override
    public void runOpMode() {

        Pose2d startPose = new Pose2d(-40, -60, Math.toRadians(180));
        Pose2d firstGround = new Pose2d(-47, -42, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        slideLift = new SlideLift(hardwareMap);
        v4Bar = hardwareMap.get(Servo.class, "v4Bar");
        intake = hardwareMap.get(CRServo.class, "intake");
        intake2 = hardwareMap.get(CRServo.class, "intake2");

        waitForStart();
        if (opModeIsActive()) {
            // ACTIONS FOR AUTO
            SlideLiftAction slidesDeposit = new SlideLiftAction(slideLift, 3890);
            SlideLiftAction slidesGround = new SlideLiftAction(slideLift, 0);

            IntakeSpinAction outtakeSample = new IntakeSpinAction(intake, intake2, 0.5, 0.5);
            IntakeSpinAction intakeSample = new IntakeSpinAction(intake, intake2, -1.0, 2);

            V4BarAction V4BarDeposit = new V4BarAction(v4Bar, 0.3);
            V4BarAction V4BarGround = new V4BarAction(v4Bar, 0.9);


            // SEQUENCE FOR DRIVING FROM STARTING POSITION TO BUCKET WHILE RAISING SLIDES
            Actions.runBlocking(drive.actionBuilder(startPose)
                    .afterTime(0, slidesDeposit) // RAISE SLIDES ACTION FOR HIGH BUCKET
                    .afterTime(0, V4BarDeposit) // V4BAR DEPOSIT POSITION
                    .strafeToLinearHeading(new Vector2d(-54, -52), Math.toRadians(225)) // BUCKET POSITION
                    .build());

            // INTAKE: 0.5 = outtake, -1.0 = intake

            // SEQUENCE FOR OUTTAKING PRELOAD, LOWERING SLIDES, DRIVING TO FIRST GROUND
            Actions.runBlocking(drive.actionBuilder(new Pose2d(-54, -52, Math.toRadians(225)))
                    .afterTime(0, outtakeSample)   // OUTTAKE
                    .afterTime(0.5, slidesGround)
                    .afterTime(2, V4BarGround)
                    .afterTime(1, drive.actionBuilder(new Pose2d(-54, -53, Math.toRadians(225)))
                            .strafeToLinearHeading(new Vector2d(-48, -42), Math.toRadians(90))
                            .build())
                    .build());

            // SEQUENCE FOR INTAKING FIRST GROUND
            Actions.runBlocking(drive.actionBuilder(new Pose2d(-48, -42, Math.toRadians(90)))
                    .afterTime(0, intakeSample)
                    .afterTime(0, drive.actionBuilder(new Pose2d(-48, -42, Math.toRadians(90)))
                            .strafeTo(new Vector2d(-48, -28))
                            .build())
                    .build());

            // SEQUENCE FOR DRIVING TO BUCKET, DEPOSITING FIRST GROUND
            Actions.runBlocking(drive.actionBuilder(new Pose2d(-48, -28, Math.toRadians(90)))
                    .afterTime(0, V4BarDeposit)
                    .afterTime(0.5, slidesDeposit)
                    .strafeTo(new Vector2d(-48, -32))
                    .strafeToLinearHeading(new Vector2d(-54, -52), Math.toRadians(225))
                    .build());

            // SEQUENCE FOR DEPOSITING FIRST GROUND, DRIVING TO SECOND GROUND
            Actions.runBlocking(drive.actionBuilder(new Pose2d(-54, -52, Math.toRadians(225)))
                    .afterTime(0, outtakeSample)   // OUTTAKE
                    .afterTime(0.5, slidesGround)
                    .afterTime(2, V4BarGround)
                    .afterTime(1, drive.actionBuilder(new Pose2d(-54, -52, Math.toRadians(225)))
                            .strafeToLinearHeading(new Vector2d(-58, -42), Math.toRadians(90))
                            .build())
                    .build());

            // SEQUENCE FOR INTAKING SECOND GROUND
            Actions.runBlocking(drive.actionBuilder(new Pose2d(-58, -42, Math.toRadians(90)))
                    .afterTime(0, intakeSample)
                    .afterTime(0, drive.actionBuilder(new Pose2d(-58, -42, Math.toRadians(90)))
                            .strafeTo(new Vector2d(-58, -28))
                            .build())
                    .build());

            // SEQUENCE FOR DRIVING TO BUCKET, DEPOSITING SECOND GROUND
            Actions.runBlocking(drive.actionBuilder(new Pose2d(-58, -28, Math.toRadians(90)))
                    .afterTime(0, V4BarDeposit)
                    .afterTime(0.5, slidesDeposit)
                    .strafeTo(new Vector2d(-58, -32))
                    .strafeToLinearHeading(new Vector2d(-54, -52), Math.toRadians(225))

                    .build());

            // DEPOSIT SECOND GROUND, DRIVE TO THIRD GROUND
            Actions.runBlocking(drive.actionBuilder(new Pose2d(-54, -52, Math.toRadians(225)))
                    .afterTime(0, outtakeSample)   // OUTTAKE
                    .afterTime(0.5, slidesGround)
                    .afterTime(2, V4BarGround)
                    .afterTime(1, drive.actionBuilder(new Pose2d(-54, -52, Math.toRadians(225)))
                            .strafeToLinearHeading(new Vector2d(-50, -23), Math.toRadians(180))
                            .build())
                    .build());

            Actions.runBlocking(drive.actionBuilder(new Pose2d(-50, -23, Math.toRadians(180)))
                    .afterTime(0, intakeSample)
                    .afterTime(1, V4BarDeposit)
                    .afterTime(2, slidesDeposit)
                    .strafeTo(new Vector2d(-58, -23))
                    .strafeToLinearHeading(new Vector2d(-54, -52), Math.toRadians(225))
                    .build());

            Actions.runBlocking(drive.actionBuilder(new Pose2d(-54, -52, Math.toRadians(225)))
                    .afterTime(0, outtakeSample)
                    .build());

        }
    }


}
