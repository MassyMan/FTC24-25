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

@Config
@Autonomous(name = "LEFT 1+3 PIDF", group = "Autonomous")
public class LeftSideAutoPIDF extends LinearOpMode {

    private SlideLift slideLift;
    private Servo v4Bar; // Servo for the v4Bar
    private CRServo intake; // Continuous Rotation Servo for intake

    // PIDF control variables
    public static double kP = 0.002;  // Proportional gain
    public static double kF = 0.3;     // Feedforward gain
    public static final int THRESHOLD = 100; // Threshold for stopping
    private static final int MAX_TICKS = 3950; // Max encoder position for the slides
    private static final double HOLD_POWER = 0.1; // Small power to hold position
    private static final double MIN_DOWN_POWER = -0.1; // Minimum power for downward movement

    public class MoveCRServoAction implements Action {
        private CRServo servo; // Change to CRServo
        private double power;
        private double duration; // Duration in seconds
        private ElapsedTime timer = new ElapsedTime();

        public MoveCRServoAction(CRServo servo, double power, double duration) {
            this.servo = servo;
            this.power = power;
            this.duration = duration;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
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

            // Check if the current position is within the threshold
            int currentPosition = slideLift.vertL.getCurrentPosition(); // Get current position of vertL
            double error = Math.abs(targetTicks - currentPosition); // Calculate the error

            // If within the threshold, return false to stop the action
            return error > THRESHOLD; // Return true to continue running
        }
    }

    public class WaitAction implements Action {
        private ElapsedTime timer = new ElapsedTime();
        private double seconds;

        public WaitAction(double seconds) {
            this.seconds = seconds;
            timer.reset();
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            return timer.seconds() < seconds; // Return true while waiting
        }
    }

    @Override
    public void runOpMode() {
        // Initialize drive and slide system
        Pose2d startPose = new Pose2d(-36, -60, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        slideLift = new SlideLift(hardwareMap);  // Initialize slide lift with hardwareMap

        // Initialize the v4Bar servo
        v4Bar = hardwareMap.get(Servo.class, "v4Bar"); // Initialize the servo
        v4Bar.setPosition(0.02077); // Move the servo to position 0.02077

        // Initialize the intake CR servo
        intake = hardwareMap.get(CRServo.class, "intake"); // Initialize the CR servo
        intake.setPower(0); // Set it to neutral initially

        // Hold the servo position during initialization
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Status", "Waiting for start");
            telemetry.addData("v4Bar Position", v4Bar.getPosition());
            telemetry.update();
        }

        waitForStart();

        if (opModeIsActive()) {
            // Autonomous sequence
            Actions.runBlocking(new SequentialAction(
                    // Drive movement
                    drive.actionBuilder(startPose)
                            .strafeTo(new Vector2d(-58, -52))
                            .turn(Math.toRadians(55))
                            .build(),
                    new SlideLiftAction(slideLift, 3400),  // Move to 3900 ticks
                    new MoveServoAction(v4Bar, 0.20), // Move v4Bar to position 0.1878
                    //new MoveCRServoAction(intake, -0.5, 1),
                    new WaitAction(1),
                    new MoveServoAction(v4Bar, 0.02077),
                    new WaitAction(5.0), // Wait for 8 seconds
                    new SlideLiftAction(slideLift, 10),  // Lower back to 10 ticks
                    drive.actionBuilder(new Pose2d(-58, -52, Math.toRadians(235)))
                            .turn(Math.toRadians(-55))
                            .strafeTo(new Vector2d(-38, -45))
                            .strafeTo(new Vector2d(-38, -22))
                            .strafeTo(new Vector2d(-36, -60))
                            .build()
            ));
        }
    }
}