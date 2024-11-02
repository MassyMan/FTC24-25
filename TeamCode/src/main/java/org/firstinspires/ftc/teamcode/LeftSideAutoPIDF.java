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
@Autonomous(name = "LEFT SIDE AUTO", group = "Autonomous")
public class LeftSideAutoPIDF extends LinearOpMode {

    private SlideLift slideLift;
    private Servo v4Bar;
    private CRServo intake;

    // PIDF control variables
    public static double kP = 0.004;
    public static double kF = 0.35;
    public static final int THRESHOLD = 80;
    private static final int MAX_TICKS = 3950;
    private static final double HOLD_POWER = 0.05;
    private static final double MIN_DOWN_POWER = -0.1;

    public class ParallelAction implements Action {
        private Action[] actions;

        public ParallelAction(Action... actions) {
            this.actions = actions;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            boolean allComplete = true;
            for (Action action : actions) {
                if (action != null) {
                    allComplete &= action.run(packet);
                }
            }
            return !allComplete;
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

            double dynamicKF = (error < 0) ? 0.05 : kF;
            double power = kP * error + dynamicKF;
            power = Range.clip(power, -1.0, 1.0);

            boolean isInThreshold = Math.abs(error) <= THRESHOLD;
            if (isInThreshold) {
                power = HOLD_POWER; // Hold power to maintain position
            } else if (power < 0) {
                power = Range.clip(power, MIN_DOWN_POWER, 1.0);
            }

            vertL.setPower(power);
            vertR.setPower(power);

            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("Current Position", currentPosition);
            telemetry.addData("Error", error);
            telemetry.addData("Power Output", power);
            telemetry.update();
        }

        public void stopSlides() {
            vertL.setPower(0);
            vertR.setPower(0);
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
            this.targetTicks = targetTicks;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            slideLift.moveSlides(targetTicks);
            return !slideLift.isAtTarget(); // Continue until the target is reached
        }
    }

    public class WaitAction implements Action {
        private final double seconds;
        private final ElapsedTime timer = new ElapsedTime();

        public WaitAction(double seconds) {
            this.seconds = seconds;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            if (timer.seconds() >= seconds) {
                return false; // Completed wait
            } else {
                telemetry.addData("Waiting", "%.2f seconds elapsed", timer.seconds());
                telemetry.update();
                return true; // Still waiting
            }
        }
    }

    @Override
    public void runOpMode() {
        Pose2d startPose = new Pose2d(-10, -60, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        slideLift = new SlideLift(hardwareMap);
        v4Bar = hardwareMap.get(Servo.class, "v4Bar");
        intake = hardwareMap.get(CRServo.class, "intake");

        waitForStart();

        if (opModeIsActive()) {
            // Move the slides to the target position while strafing
            ParallelAction moveAndStrafe = new ParallelAction(
                    drive.actionBuilder(startPose)
                            .strafeTo(new Vector2d(-10, -50))
                            .build(),
                    new SlideLiftAction(slideLift, 1600)
            );

            Actions.runBlocking(moveAndStrafe);

            // Wait for 3 seconds, blocking all other actions
            Actions.runBlocking(new WaitAction(3));

            // Lower the slides back down to 0
            SlideLiftAction lowerSlides = new SlideLiftAction(slideLift, 0);
            Actions.runBlocking(lowerSlides);

            // Stop the slides after reaching the target position
            slideLift.stopSlides();

            // Stop the drivetrain
            drive.stop(); // Call the stop method to stop the drivetrain

            // Continue with other autonomous actions if needed
            Actions.runBlocking(drive.actionBuilder(new Pose2d(-10, -50, Math.toRadians(90)))
                    .strafeTo(new Vector2d(-45, -39))
                    .build());
        }
    }
}
