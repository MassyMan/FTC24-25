package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
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

import java.util.Vector;

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
    private static final double MIN_DOWN_POWER = -0.8;

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

            // Check if within the threshold
            if ((Math.abs(error) <= THRESHOLD + 30) && targetPosition != 0) {
                vertL.setPower(HOLD_POWER);
                vertR.setPower(HOLD_POWER);
                telemetry.addData("Slide Lift", "Holding at Position: %d", currentPosition);
                telemetry.addData("Slide Lift", "Holding Power Applied: %.2f", HOLD_POWER);
                telemetry.update();
                return;
            } else if ((targetPosition == 0) && currentPosition <= THRESHOLD){
                vertL.setPower(0);
                vertR.setPower(0);
                telemetry.addData("Slide Lift", "Stopping power, gravity pulling to 0");
                telemetry.update();
                return;
            }

            // Calculate power based on error
            double power = kP * error + kF;
            power = Range.clip(power, -1.0, 1.0);

            // Limit downward speed
            if (power < 0 && currentPosition <= 50) {
                power = 0;
            } else if (power < 0) {
                power = Math.max(power, MIN_DOWN_POWER);
            }

            // Set motor power
            vertL.setPower(power);
            vertR.setPower(power);

            telemetry.addData("Slide Lift", "Target Position: %d", (int) targetPosition);
            telemetry.addData("Current Position", currentPosition);
            telemetry.addData("Error", error);
            telemetry.addData("Power Output", power);
            telemetry.addData("Is At Target", isAtTarget());
            telemetry.update();
        }

        public boolean isAtTarget() {
            return Math.abs(targetPosition - vertL.getCurrentPosition()) <= THRESHOLD+10; // +10 to fix jittering back out glitch
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
        private boolean timerStarted = false;

        public IntakeSpinAction(CRServo intake, CRServo intake2, double power, double duration) {
            this.intake = intake;
            this.intake2 = intake2;
            this.power = power;
            this.duration = duration;
            this.timer = new ElapsedTime();
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            if (!timerStarted) {
                timer.reset();
                timerStarted = true;
            }

            if (timer.seconds() < duration) {
                intake.setPower(power);
                intake2.setPower(-power);
                telemetry.addData("Intake", "Running at power: %.2f for %.2f seconds", power, duration);
                telemetry.update();
                return true;
            } else {
                intake.setPower(0);
                intake2.setPower(0);
                telemetry.addData("Intake", "Stopped after duration: %.2f seconds", duration);
                telemetry.update();
                return false;
            }
        }
    }

    @Override
    public void runOpMode() {
        Pose2d startPose = new Pose2d(-40, -60, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        slideLift = new SlideLift(hardwareMap);
        v4Bar = hardwareMap.get(Servo.class, "v4Bar");
        intake = hardwareMap.get(CRServo.class, "intake");
        intake2 = hardwareMap.get(CRServo.class, "intake2");

        waitForStart();

        if (opModeIsActive()) {
            // Start raising slides while strafing
            SlideLiftAction raiseSlides = new SlideLiftAction(slideLift, 3800);
            Actions.runBlocking(drive.actionBuilder(startPose)
                    .afterTime(0, raiseSlides)
                    .afterTime(0, new V4BarAction(v4Bar, 0.3))
                    .strafeToSplineHeading(new Vector2d(-58, -50), Math.toRadians(225))
                    .build());

            // Check if slides are within the target position
            while (opModeIsActive() && !raiseSlides.slideLift.isAtTarget()) {
                // Update telemetry or perform other actions if needed
                telemetry.addData("Raising Slides", "Current Position: %d", slideLift.vertL.getCurrentPosition());
                telemetry.update();
                idle(); // Prevent CPU overuse
            }

            // Now run the actions for intake, v4Bar, and moving slides back down
            Actions.runBlocking(drive.actionBuilder(startPose)
                    .afterTime(0, new IntakeSpinAction(intake, intake2, 0.8, 0.5))
                    .afterTime(0.8, new V4BarAction(v4Bar, 0.25)) // Slight delay after intake
                    .afterTime(0, new SlideLiftAction(slideLift, 0)) // Lower slides to 0
                    .setReversed(true)
                    .strafeToLinearHeading(new Vector2d(-50, -42), Math.toRadians(180))
                    .build());
        }
    }

}
