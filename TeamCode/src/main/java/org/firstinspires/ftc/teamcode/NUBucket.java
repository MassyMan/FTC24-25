package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Autonomous.V4Bar;

import java.util.Vector;

@Config
@Autonomous(name = "NU Bucket", group = "Autonomous")
public class NUBucket extends LinearOpMode {

    private SlideLift slideLift;
    private Servo v4Bar;
    private CRServo intakeL, intakeR, slidL, slidR;
    private ExtendoMove extendoMove;

    // PIDF control variables
    public static double kP = 0.08;
    public static double kF = 0.2;
    public static final int THRESHOLD = 80;
    private static final double HOLD_POWER = 0.15;
    private static final double MIN_DOWN_POWER = -0.99;
    private static final int MAX_TICKS = 2000;
    private static final double MIN_EXTENDO = 0;
    private static final double MAX_EXTENDO = 15400;
    private static final double EXTENDO_SPEED = -1.0;
    private static final double EXTENDO_TOLERANCE = 200;


    public class ExtendoMove {
        private DcMotor extendoEncoder;
        private double targetExtendo = 0;

        public ExtendoMove(HardwareMap hardwareMap) {
            slidL = hardwareMap.get(CRServo.class, "slidL");
            slidR = hardwareMap.get(CRServo.class, "slidR");
            extendoEncoder = hardwareMap.get(DcMotorEx.class, "vertR");
            extendoEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extendoEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        public void moveExtendo(double targetExtendo) {
            double currentExtendo = -extendoEncoder.getCurrentPosition();
            double extendoError = Math.abs(targetExtendo - currentExtendo);
            this.targetExtendo = Range.clip(targetExtendo, MIN_EXTENDO, MAX_EXTENDO);

            if (targetExtendo > currentExtendo) {
                slidL.setPower(EXTENDO_SPEED);
                slidR.setPower(-EXTENDO_SPEED);
            } else {
                slidL.setPower(-EXTENDO_SPEED);
                slidR.setPower(EXTENDO_SPEED);
            }
        }

        public boolean extendoAtTarget() {
            return Math.abs(targetExtendo + extendoEncoder.getCurrentPosition()) <= EXTENDO_TOLERANCE; // Threshold for error
        }
    }

    public class ExtendoAction implements Action {
        private ExtendoMove extendoMove;
        private double targetExtendo;

        public ExtendoAction(ExtendoMove extendoMove, double targetExtendo) {
            this.extendoMove = extendoMove;
            this.targetExtendo = Range.clip(targetExtendo, 0, MAX_EXTENDO);
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            extendoMove.moveExtendo(targetExtendo);
            boolean extendoAtTarget = extendoMove.extendoAtTarget();
            if (extendoAtTarget) {
                slidL.setPower(0);
                slidR.setPower(0);
            }
            telemetry.addData("ExtendoAction", "At Target: %b, Target Position: %.2f", extendoAtTarget, targetExtendo);
            telemetry.update();
            return !extendoAtTarget;
        }
    }

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

            if ((currentPosition <= 1) && (power < 0) && targetPosition == 0) {
                stopSlides();
                power = 0;
                telemetry.addData("Slide Lift", "Stopping power, gravity pulling to 0");
                telemetry.update();
                if (currentPosition < 0){
                    vertL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                }
            }

            vertL.setPower(power);
            vertR.setPower(power);

            if (Math.abs(error) <= THRESHOLD && targetPosition != 0) {
                vertL.setPower(HOLD_POWER);
                vertR.setPower(HOLD_POWER);
                telemetry.addData("Slide Lift", "Holding at Position: %d", currentPosition);
                telemetry.update();
            }

            telemetry.addData("Slide Lift", "Target Position: %d", (int) targetPosition);
            telemetry.addData("Current Position", currentPosition);
            telemetry.addData("Error", error);
            telemetry.addData("Power Output", power);
            telemetry.addData("Is At Target", isAtTarget());
            telemetry.update();
        }

        public void stopSlides() {
            if (targetPosition != 0) {
                vertL.setPower(HOLD_POWER);
                vertR.setPower(HOLD_POWER);
            } else {
                vertL.setPower(0);
                vertR.setPower(0);
            }

        }

        public boolean isAtTarget() {
            if (targetPosition != 0) {
                return Math.abs(targetPosition - vertL.getCurrentPosition()) <= THRESHOLD;
            } else {
                return vertL.getCurrentPosition() <= 0;
            }

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
            if (isAtTarget) {
                slideLift.stopSlides();
            }
            telemetry.addData("SlideLiftAction", "At Target: %b, Target Ticks: %.2f", isAtTarget, targetTicks);
            telemetry.update();
            return !isAtTarget;
        }
    }

    public class IntakeSpinAction implements Action {
        private CRServo intakeL;
        private CRServo intakeR;
        private double power;
        private double duration;
        private ElapsedTime timer;
        private boolean timerStarted = false; // Flag to track if the timer has started

        public IntakeSpinAction(CRServo intakeL, CRServo intakeR, double power, double duration) {
            this.intakeL = intakeL;
            this.intakeR = intakeR;
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
                intakeL.setPower(power);
                intakeR.setPower(-power);
                telemetry.addData("Intake", "Running at power: %.2f for %.2f seconds", power, duration);
                telemetry.update();
                return true;
            } else {
                // Stop the intake and mark the action as complete
                intakeL.setPower(0);
                intakeR.setPower(0);
                telemetry.addData("Intake", "Stopped after duration: %.2f seconds", duration);
                telemetry.update();
                timerStarted = false;
                timer.reset();
                return false;
            }
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

    @Override
    public void runOpMode() {
        Pose2d startPose = new Pose2d(-40, -58, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        slideLift = new SlideLift(hardwareMap);
        extendoMove = new ExtendoMove(hardwareMap);
        v4Bar = hardwareMap.get(Servo.class, "v4Bar");
        intakeL = hardwareMap.get(CRServo.class, "intakeL");
        intakeR = hardwareMap.get(CRServo.class, "intakeR");

        waitForStart();
        if (opModeIsActive()) {
            SlideLiftAction slidesSpecimen = new SlideLiftAction(slideLift, 750);
            SlideLiftAction slidesGround = new SlideLiftAction(slideLift, 0);
            SlideLiftAction slidesBucket = new SlideLiftAction(slideLift, 1900);

            V4BarAction V4BarDeposit = new V4BarAction(v4Bar, 0.17);
            V4BarAction V4BarGround = new V4BarAction(v4Bar, 0.72);
            V4BarAction V4BarHP = new V4BarAction(v4Bar, 0.35);

            ExtendoAction ExtendoIntake = new ExtendoAction(extendoMove, 3000);
            ExtendoAction ExtendoBiggie = new ExtendoAction(extendoMove, 9000);
            ExtendoAction ExtendoRetract = new ExtendoAction(extendoMove, -25);

            IntakeSpinAction IntakeSample = new IntakeSpinAction(intakeL, intakeR, -1, 1);
            IntakeSpinAction IntakeSpecimen = new IntakeSpinAction(intakeL, intakeR, -1, 0.2);
            IntakeSpinAction OuttakeSample = new IntakeSpinAction(intakeL, intakeR, 1.0, 0.2);

            Actions.runBlocking(drive.actionBuilder(startPose)
                    .afterTime(0, slidesBucket)
                    .afterTime(0, V4BarDeposit)
                    .afterTime(1.6, OuttakeSample)
                    .afterTime(1.9, slidesGround)
                    .afterTime(2.75, ExtendoIntake)
                    .afterTime(3, V4BarGround)


                    .strafeToLinearHeading(new Vector2d(-57, -48), Math.toRadians(225), // BUCKET POSITION
                            new TranslationalVelConstraint(60),
                            new ProfileAccelConstraint(-60, 60))
                    .waitSeconds(1)
                    .strafeToLinearHeading(new Vector2d(-49, -45), Math.toRadians(90),
                            new TranslationalVelConstraint(40),
                            new ProfileAccelConstraint(-40, 40))
                    .build());

            Actions.runBlocking(drive.actionBuilder(new Pose2d(-49, -45, Math.toRadians(90)))
                    .afterTime(0, IntakeSample)
                    .afterTime(1, V4BarDeposit)
                    .afterTime(1, ExtendoRetract)
                    .afterTime(1, slidesBucket)
                    .afterTime(2.8, OuttakeSample)

                    .strafeToLinearHeading(new Vector2d(-49, -34), Math.toRadians(90), // FIRST BLOCK POSITION
                            new TranslationalVelConstraint(40),
                            new ProfileAccelConstraint(-40, 40))
                    .strafeToLinearHeading(new Vector2d(-54, -48), Math.toRadians(225), // BUCKET POSITION
                            new TranslationalVelConstraint(60),
                            new ProfileAccelConstraint(-60, 60))
                    .build());

            Actions.runBlocking(drive.actionBuilder(new Pose2d(-54, -48, Math.toRadians(225)))
                    .afterTime(0, slidesGround)
                    .afterTime(0.7, ExtendoIntake)
                    .afterTime(0.7, V4BarGround)
                    .afterTime(1.5, IntakeSample)
                    .afterTime(2.5, ExtendoRetract)
                    .afterTime(2.5, V4BarDeposit)
                    .afterTime(2.5, slidesBucket)
                    .afterTime(3.9, OuttakeSample)

                    .waitSeconds(0.3)
                    .strafeToLinearHeading(new Vector2d(-58.5, -45), Math.toRadians(90),
                            new TranslationalVelConstraint(60),
                            new ProfileAccelConstraint(-60, 60))
                    .strafeToLinearHeading(new Vector2d(-58.5, -33), Math.toRadians(90), // SECOND BLOCK POSITION
                            new TranslationalVelConstraint(20),
                            new ProfileAccelConstraint(-20, 20))
                    .waitSeconds(0.25)
                    .strafeToLinearHeading(new Vector2d(-54, -48), Math.toRadians(225), // BUCKET POSITION
                            new TranslationalVelConstraint(40),
                            new ProfileAccelConstraint(-40, 40))


                    .build());

            Actions.runBlocking(drive.actionBuilder(new Pose2d(-54, -48, Math.toRadians(225)))
                    .afterTime(0, slidesGround)
                            .waitSeconds(0.3)

                    .strafeToLinearHeading(new Vector2d(-40, -34), Math.toRadians(180), // FIRST BLOCK POSITION
                            new TranslationalVelConstraint(60),
                            new ProfileAccelConstraint(-60, 60))
                    .strafeToLinearHeading(new Vector2d(34, -34), Math.toRadians(180), // FIRST BLOCK POSITION
                            new TranslationalVelConstraint(100),
                            new ProfileAccelConstraint(-100, 100))
                    .setReversed(false)
                    .splineToLinearHeading(new Pose2d(34, -11, Math.toRadians(270)), Math.PI / 2, // Bar position
                            new TranslationalVelConstraint(100),
                            new ProfileAccelConstraint(-100, 100))

                    .strafeToLinearHeading(new Vector2d(44, -11), Math.toRadians(180), // FIRST BLOCK POSITION
                            new TranslationalVelConstraint(100),
                            new ProfileAccelConstraint(-100, 100))
                    .strafeToLinearHeading(new Vector2d(44, -45), Math.toRadians(180), // FIRST BLOCK POSITION
                            new TranslationalVelConstraint(100),
                            new ProfileAccelConstraint(-100, 100))







                    .build());








        }
    }
}