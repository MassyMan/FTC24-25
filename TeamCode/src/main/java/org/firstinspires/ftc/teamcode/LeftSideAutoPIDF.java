package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
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

@Config
@Autonomous(name = "LEFT 1+3 PIDF", group = "Autonomous")
public class LeftSideAutoPIDF extends LinearOpMode {

    private SlideLift slideLift;
    private Servo v4Bar; // Servo for the v4Bar
    private CRServo intake; // Continuous Rotation Servo for intake

    // PIDF control variables
    public static double kP = 0.0023;  // Proportional gain
    public static double kF = 0.32;     // Feedforward gain
    public static final int THRESHOLD = 80; // Threshold for stopping
    private static final int MAX_TICKS = 3950; // Max encoder position for the slides
    private static final double HOLD_POWER = 0.05; // Small power to hold position
    private static final double MIN_DOWN_POWER = -0.1; // Minimum power for downward movement

    public class MoveCRServoAction implements Action {
        private CRServo servo;
        private double power;
        private double duration; // Duration in seconds
        private ElapsedTime timer = new ElapsedTime();
        private boolean timerStarted = false; // Flag to check if the timer is started

        public MoveCRServoAction(CRServo servo, double power, double duration) {
            this.servo = servo;
            this.power = power;
            this.duration = duration;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            // Start the timer only when the action is first executed
            if (!timerStarted) {
                timer.reset();
                timerStarted = true;
            }

            // If the timer hasn't exceeded the duration, keep running the servo
            if (timer.seconds() < duration) {
                servo.setPower(power); // Set the power
                return true; // Continue running this action
            } else {
                servo.setPower(0); // Stop the servo
                return false; // Complete this action
            }
        }
    }

    public class MoveServoAction implements Action {
        private Servo servo;
        private double position;

        public MoveServoAction(Servo servo, double position) {
            this.servo = servo;
            this.position = position;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            servo.setPosition(position); // Set the servo to the desired position
            return false; // Immediately complete this action
        }
    }

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
                    allComplete &= action.run(packet); // Run each action
                }
            }
            return !allComplete; // Return true if any action is still running
        }
    }

    // SlideLift class for PIDF control
    public class SlideLift {
        private DcMotorEx vertL;
        private DcMotorEx vertR;
        private double targetPosition = 0;

        public SlideLift(HardwareMap hardwareMap) {
            // Initialize the motors using hardwareMap
            vertL = hardwareMap.get(DcMotorEx.class, "vertL");
            vertR = hardwareMap.get(DcMotorEx.class, "vertR");

            vertL.setDirection(DcMotorSimple.Direction.FORWARD);
            vertR.setDirection(DcMotorSimple.Direction.FORWARD);

            vertL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            vertR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

            // Reset encoders and set to RUN_WITHOUT_ENCODER
            vertL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            vertR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            vertL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            vertR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

            // Telemetry output for encoder position after reset
            telemetry.addData("Encoder Reset - vertL Position", vertL.getCurrentPosition());
            telemetry.update();
        }

        public void moveSlides(double targetTicks) {
            targetPosition = Range.clip(targetTicks, 0, MAX_TICKS); // Clip target within range
            int currentPosition = vertL.getCurrentPosition();
            double error = targetPosition - currentPosition;

            // Adjust feedforward based on error direction (reduce when lowering)
            double dynamicKF = (error < 0) ? 0.05 : kF;
            double power = kP * error + dynamicKF; // Calculate power using PIDF
            power = Range.clip(power, -1.0, 1.0); // Clip power to [-1, 1]

            // Check if within threshold and adjust power accordingly
            boolean isInThreshold = Math.abs(error) <= THRESHOLD;
            if (isInThreshold) {
                // Apply a small hold power
                power = HOLD_POWER; // Hold position
                vertL.setPower(power);
                vertR.setPower(-power);
            } else {
                // Set power to both motors
                if (power < 0) {
                    // Ensure a minimum power when moving down
                    power = Range.clip(power, MIN_DOWN_POWER, 1.0);
                }
                vertL.setPower(power);  // Positive power for raising
                vertR.setPower(-power); // Negative power for raising
            }

            // Ensure the slides go fully to 0
            if (targetPosition == 0 && currentPosition <= 50) {
                vertL.setPower(0);
                vertR.setPower(0);
            }

            // Telemetry
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
            // Move the slides
            slideLift.moveSlides(targetTicks);

            // Get current position of vertL
            int currentPosition = slideLift.vertL.getCurrentPosition();
            double error = Math.abs(targetTicks - currentPosition); // Calculate the error

            // If the target is 0 or close to 0, stop the slides completely
            if (targetTicks == 0 && currentPosition <= 50) {
                slideLift.stopSlides(); // Stop the slides
                return false; // Action completes immediately
            }

            // If within the threshold, return false to stop the action
            return error > THRESHOLD; // Return true to continue running
        }
    }

    // Action to stop the slides completely
    public class StopSlideAction implements Action {
        private SlideLift slideLift;

        public StopSlideAction(SlideLift slideLift) {
            this.slideLift = slideLift;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            slideLift.stopSlides(); // Stop both motors
            return false; // Action completes immediately
        }
    }

    public class WaitAction implements Action {
        private ElapsedTime timer = new ElapsedTime();
        private double seconds;
        private boolean timerStarted = false; // Add a flag to track if the timer has been started

        public WaitAction(double seconds) {
            this.seconds = seconds;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            // Start the timer the first time this method is called
            if (!timerStarted) {
                timer.reset();
                timerStarted = true; // Mark the timer as started
            }

            // Continue waiting until the timer reaches the specified duration
            return timer.seconds() < seconds; // Return true while waiting
        }
    }

    public class SetCRServoPowerAction implements Action {
        private CRServo servo;
        private double power;

        public SetCRServoPowerAction(CRServo servo, double power) {
            this.servo = servo;
            this.power = power;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            servo.setPower(power); // Set the power of the CRServo
            return false; // Complete this action immediately
        }
    }



    @Override
    public void runOpMode() {
        // Initialize drive and slide system
        Pose2d startPose = new Pose2d(-36, -60, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        slideLift = new SlideLift(hardwareMap);
        v4Bar = hardwareMap.get(Servo.class, "v4Bar");
        intake = hardwareMap.get(CRServo.class, "intake");

        // Wait for the start signal
        waitForStart();

        // Check if the OpMode is active -52 -58 and 55 deg for angleDROP 0.907 0.1967 0.4
        if (opModeIsActive()) {
            // Run autonomous sequence
            Actions.runBlocking(new SequentialAction(
                    drive.actionBuilder(startPose)
                            .strafeToLinearHeading(new Vector2d(-58, -52), Math.toRadians(230))
                           // .afterDisp(1, new SlideLiftAction(slideLift, 3600))
                            .build(),
                    new SlideLiftAction(slideLift, 3600),
                    new MoveServoAction(v4Bar, 0.4), // Move v4Bar to position 0.15
                    new MoveCRServoAction(intake, -0.5, 0.5), // Outtake preload
                    new MoveServoAction(v4Bar, 0.1967), // Move v4Bar back
                    new SlideLiftAction(slideLift, 0), // Lower slides

                    // 1ST YELLOW OFF FLOOR

                    drive.actionBuilder(new Pose2d(-58, -52, Math.toRadians(230)))
                            .strafeToLinearHeading(new Vector2d(-37, -45), Math.toRadians(180))
                            .build(),
                    new MoveServoAction(v4Bar, 0.907), // Lower v4Bar to GROUND
                    drive.actionBuilder(new Pose2d(-37,-45, Math.toRadians(180)))
                            .strafeTo(new Vector2d(-37, -23.5))
                            .build(),
                    new SetCRServoPowerAction(intake, 1),

                    drive.actionBuilder(new Pose2d(-37, -23.5, Math.toRadians(180)))
                            .strafeTo(new Vector2d(-44, -23.5))
                            .build(),

                    new SetCRServoPowerAction(intake,0),
                    new MoveServoAction(v4Bar, 0.1967), // Move v4Bar TO TOP

                    drive.actionBuilder(new Pose2d(-44, -23.5, Math.toRadians(180)))
                            .strafeToLinearHeading(new Vector2d(-58, -52), Math.toRadians(230))
                            .build(),

                    new SlideLiftAction(slideLift, 3600), // Move slides to 3600 ticks
                    new MoveServoAction(v4Bar, 0.4), // Move v4Bar to position 0.15
                    new MoveCRServoAction(intake, -0.7, 0.5), // Outtake preload
                    new MoveServoAction(v4Bar, 0.1967), // Move v4Bar back
                    new SlideLiftAction(slideLift, 0), // Lower slides

                    // 2ND YELLOW OFF FLOOR

                    drive.actionBuilder(new Pose2d(-58, -52, Math.toRadians(230)))
                            .strafeToLinearHeading(new Vector2d(-40, -23.5), Math.toRadians(180))
                            .build(),

                    new MoveServoAction(v4Bar, 0.907), // Move v4Bar TO GROUND
                    new SetCRServoPowerAction(intake,1),
                    new WaitAction(0.8),

                    // TODO: Simplify these positions with variables for the pose instead of manual coordinates
                    // TODO: Ask chatgpt about it rehehe

                    drive.actionBuilder(new Pose2d(-40, -23.5, Math.toRadians(180)))
                            .strafeTo(new Vector2d(-53, -23.5)) // Strafe to
                            .build(),

                    new MoveServoAction(v4Bar, 0.1967), // Move v4Bar back
                    new SetCRServoPowerAction(intake,0),

                    drive.actionBuilder(new Pose2d(-53, -23.5, Math.toRadians(180)))
                            .strafeToLinearHeading(new Vector2d(-58, -52), Math.toRadians(230))
                            .build(),

                    new SlideLiftAction(slideLift, 3600), // Move slides to 3600 ticks
                    new MoveServoAction(v4Bar, 0.4), // Move v4Bar to position 0.15
                    new MoveCRServoAction(intake, -0.5, 0.5), // Outtake preload
                    new MoveServoAction(v4Bar, 0.1967), // Move v4Bar back
                    new SlideLiftAction(slideLift, 0) // Lower slides

                    /* 3RD YELLOW OFF FLOOR **FIX SERVO POS**

                    drive.actionBuilder(new Pose2d(-58, -52, Math.toRadians(235)))
                            .turn(Math.toRadians(-55))
                            .strafeTo(new Vector2d(-53, -23.5)) // Strafe to
                            .build(),

                    new MoveServoAction(v4Bar, 0.708), // Move v4Bar TO GROUND
                    new SetCRServoPowerAction(intake,1),
                    new WaitAction(0.8),

                    // TODO: Simplify these positions with variables for the pose instead of manual coordinates
                    // TODO: Ask chatgpt about it rehehe

                    drive.actionBuilder(new Pose2d(-53, -23.5, Math.toRadians(180)))
                            .strafeTo(new Vector2d(-61, -23.5)) // Strafe to
                            .build(),

                    new MoveServoAction(v4Bar, 0.02077), // Move v4Bar back
                    new SetCRServoPowerAction(intake,0),

                    drive.actionBuilder(new Pose2d(-61, -23.5, Math.toRadians(180)))
                            .strafeTo(new Vector2d(-58, -52))
                            .turn(Math.toRadians(55))
                            .build(),

                    new SlideLiftAction(slideLift, 3600), // Move slides to 3600 ticks
                    new MoveServoAction(v4Bar, 0.15), // Move v4Bar to position 0.15
                    new MoveCRServoAction(intake, -0.5, 0.5), // Outtake preload
                    new MoveServoAction(v4Bar, 0.02077), // Move v4Bar back
                    new SlideLiftAction(slideLift, 0), // Lower slides

                    drive.actionBuilder(new Pose2d(-58, -52, Math.toRadians(255)))
                            .turn(Math.toRadians(-55))
                            .strafeTo(new Vector2d(36, -60))
                            .build()
                */
            ));
        }
    }
}