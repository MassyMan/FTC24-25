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

import java.util.Vector;

@Config
@Autonomous(name = "RIGHT Specs", group = "Autonomous")
public class AUTORightSpecs extends LinearOpMode {

    private SlideLift slideLift;
    private Servo v4Bar;
    private CRServo intake, intake2, slidL, slidR;
    private ExtendoMove extendoMove;

    // PIDF control variables
    public static double kP = 0.005;
    public static double kF = 0.2;
    public static final int THRESHOLD = 80;
    private static final double HOLD_POWER = 0.15;
    private static final double MIN_DOWN_POWER = -0.90;
    private static final int MAX_TICKS = 3830;
    private static final double MAX_EXTENDO = 16500;

    public class ExtendoMove {
        private DcMotor extendoEncoder;
        private double targetExtendo = 0;

        public ExtendoMove(HardwareMap hardwareMap) {
            slidL = hardwareMap.get(CRServo.class, "slidL");
            slidR = hardwareMap.get(CRServo.class, "slidR");
            extendoEncoder = hardwareMap.get(DcMotorEx.class, "leftBack");
            extendoEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extendoEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        public void moveExtendo(double targetExtendo) {
            double currentExtendo = -extendoEncoder.getCurrentPosition();
            double extendoError = Math.abs(targetExtendo - currentExtendo);
            this.targetExtendo = Range.clip(targetExtendo, 0, MAX_EXTENDO);

            if (targetExtendo > currentExtendo) {
                slidL.setPower(1.0);
                slidR.setPower(-1.0);
            } else {
                slidL.setPower(-1.0);
                slidR.setPower(1.0);
            }
        }

        public boolean extendoAtTarget() {
            return Math.abs(targetExtendo + extendoEncoder.getCurrentPosition()) <= 500; // Threshold for error
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

            if ((currentPosition <= 100) && (power < 0) && targetPosition == 0) {
                vertL.setPower(0);
                vertR.setPower(0);
                power = 0;
                telemetry.addData("Slide Lift", "Stopping power, gravity pulling to 0");
                telemetry.update();
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
            return Math.abs(targetPosition - vertL.getCurrentPosition()) <= THRESHOLD;
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
        Pose2d startPose = new Pose2d(9, -60, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        slideLift = new SlideLift(hardwareMap);
        extendoMove = new ExtendoMove(hardwareMap);
        v4Bar = hardwareMap.get(Servo.class, "v4Bar");
        intake = hardwareMap.get(CRServo.class, "intake");
        intake2 = hardwareMap.get(CRServo.class, "intake2");

        waitForStart();
        if (opModeIsActive()) {
            SlideLiftAction slidesDeposit = new SlideLiftAction(slideLift, 4000);
            SlideLiftAction slidesSpecimen = new SlideLiftAction(slideLift, 1600);
            SlideLiftAction slidesGround = new SlideLiftAction(slideLift, 0);

            V4BarAction V4BarDeposit = new V4BarAction(v4Bar, 0.25);
            V4BarAction V4BarGround = new V4BarAction(v4Bar, 0.8);
            V4BarAction V4BarHP = new V4BarAction(v4Bar, 0.5);
            V4BarAction V4BarSpecimen = new V4BarAction(v4Bar, 0.43);

            ExtendoAction ExtendoIntake = new ExtendoAction(extendoMove, 2000);
            ExtendoAction ExtendoRetract = new ExtendoAction(extendoMove, 0);

            IntakeSpinAction IntakeSample = new IntakeSpinAction(intake, intake2, -1, 1.5);
            IntakeSpinAction IntakeSpecimen = new IntakeSpinAction(intake, intake2, -1, 0.2);
            IntakeSpinAction OuttakeSample = new IntakeSpinAction(intake, intake2, 0.5, 0.2);

            Actions.runBlocking(drive.actionBuilder(startPose)
                    .afterTime(0, slidesSpecimen)
                    .afterTime(1, slidesGround)
                    .afterTime(0, V4BarDeposit)
                    .afterTime(2, V4BarGround)
                    .afterTime(3.0, IntakeSample)
                    .afterTime(1.85, ExtendoIntake)
                    .afterTime(3.8, V4BarHP)
                    .afterTime(4.5, OuttakeSample)
                    .afterTime(5, V4BarGround)
                    .afterTime(5, IntakeSample)
                    .afterTime(6.5, V4BarSpecimen)
                    .afterTime(6.5, ExtendoRetract)
                    .afterTime(8.0, OuttakeSample)
                    .afterTime(8.5, IntakeSpecimen)

                    .strafeTo(new Vector2d(9, -32),
                            new TranslationalVelConstraint(55),
                            new ProfileAccelConstraint(-55, 55))
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(48, -36, Math.toRadians(76)), -5,
                            new TranslationalVelConstraint(35),
                             new ProfileAccelConstraint(-35, 35))
                    .strafeToLinearHeading(new Vector2d(40, -40), Math.toRadians(290),
                            new TranslationalVelConstraint(80),
                             new ProfileAccelConstraint(-80, 80))
                    .strafeToLinearHeading(new Vector2d(58, -45), Math.toRadians(90),
                            new TranslationalVelConstraint(80),
                            new ProfileAccelConstraint(-80, 80))
                    .strafeTo(new Vector2d(58.5, -36),
                            new TranslationalVelConstraint(10),
                            new ProfileAccelConstraint(-10, 10))
                    .strafeToLinearHeading(new Vector2d(48, -43), Math.toRadians(270),
                            new TranslationalVelConstraint(40),
                            new ProfileAccelConstraint(-40, 40))
                    .strafeTo(new Vector2d(48, -55),
                            new TranslationalVelConstraint(20),
                            new ProfileAccelConstraint(-20, 20))

                    .build());

            Actions.runBlocking(drive.actionBuilder(new Pose2d(48, -55, Math.toRadians(270)))
                    .afterTime(0, slidesSpecimen)
                    .afterTime(0.5, V4BarDeposit)
                    .afterTime(3.5, slidesGround)

                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(7, -38, Math.toRadians(90)), -11,
                            new TranslationalVelConstraint(30),
                            new ProfileAccelConstraint(-30, 30))
                    .strafeTo(new Vector2d(7, -32),
                            new TranslationalVelConstraint(20),
                            new ProfileAccelConstraint(-20, 20))
                    .build());

            Actions.runBlocking(drive.actionBuilder(new Pose2d(7, -32, Math.toRadians(90)))
                    .afterTime(1, V4BarSpecimen)
                    .afterTime(2.5, IntakeSpecimen)
                    .strafeToLinearHeading(new Vector2d(43, -46), Math.toRadians(270),
                            new TranslationalVelConstraint(60),
                            new ProfileAccelConstraint(-60, 60))
                    .strafeTo(new Vector2d(43, -55),
                            new TranslationalVelConstraint(20),
                            new ProfileAccelConstraint(-20, 20))
                    .build());

            Actions.runBlocking(drive.actionBuilder(new Pose2d(43, -55, Math.toRadians(270)))
                    .afterTime(0, slidesSpecimen)
                    .afterTime(0.5, V4BarDeposit)
                    .afterTime(3.5, slidesGround)

                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(5, -38, Math.toRadians(90)), -11,
                            new TranslationalVelConstraint(30),
                            new ProfileAccelConstraint(-30, 30))
                    .strafeTo(new Vector2d(5, -32),
                            new TranslationalVelConstraint(20),
                            new ProfileAccelConstraint(-20, 20))
                    .build());

            Actions.runBlocking(drive.actionBuilder(new Pose2d(5, -32, Math.toRadians(90)))
                    .afterTime(1, V4BarSpecimen)
                    .afterTime(2.5, IntakeSpecimen)
                    .strafeToLinearHeading(new Vector2d(43, -46), Math.toRadians(270),
                            new TranslationalVelConstraint(70),
                            new ProfileAccelConstraint(-70, 70))
                    .strafeTo(new Vector2d(43, -55),
                            new TranslationalVelConstraint(20),
                            new ProfileAccelConstraint(-20, 20))
                    .build());

            Actions.runBlocking(drive.actionBuilder(new Pose2d(43, -55, Math.toRadians(270)))
                    .afterTime(0, slidesSpecimen)
                    .afterTime(0.5, V4BarDeposit)
                    .afterTime(3.5, slidesGround)

                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(3, -38, Math.toRadians(90)), -11,
                            new TranslationalVelConstraint(30),
                            new ProfileAccelConstraint(-30, 30))
                    .strafeTo(new Vector2d(3, -32),
                            new TranslationalVelConstraint(20),
                            new ProfileAccelConstraint(-20, 20))
                    .build());

            Actions.runBlocking(drive.actionBuilder(new Pose2d(3, -32, Math.toRadians(90)))
                    .strafeToSplineHeading(new Vector2d(45, -45), Math.toRadians(140),
                            new TranslationalVelConstraint(100),
                            new ProfileAccelConstraint(-100, 100))
                    .build());


        }
    }
}
