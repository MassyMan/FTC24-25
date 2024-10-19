package org.firstinspires.ftc.teamcode;

// RR-specific imports
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.util.ElapsedTime;
// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.acmerobotics.dashboard.config.Config;

@Config
@Autonomous(name = "LEFT 1+3", group = "Autonomous")
public class LeftSideAuto extends LinearOpMode {

    private SlideLift slideLift;

    // SlideLift class for managing the lift system
    public class SlideLift {
        private DcMotorEx vertL;
        private DcMotorEx vertR;
        private static final int TOLERANCE = 120;  // Define a tolerance of 120 encoder ticks
        private static final int MAX_TICKS = 3900;  // Maximum limit for encoder ticks
        private static final double LOW_POWER = 0.2;  // Low power to prevent overshooting
        private boolean isAtTarget = false;  // Flag to indicate if the slides are at the target position

        public SlideLift(HardwareMap hardwareMap) {
            vertL = hardwareMap.get(DcMotorEx.class, "vertL");
            vertR = hardwareMap.get(DcMotorEx.class, "vertR");

            // Set zero power behavior to brake
            vertL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            vertR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

            // Set motor directions to spin in the same direction
            vertL.setDirection(DcMotorSimple.Direction.FORWARD);
            vertR.setDirection(DcMotorSimple.Direction.REVERSE);

            // Reset encoder and set mode to run using encoder for both motors
            vertL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            vertR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            vertL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            vertR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }

        // Method to raise the slides using target ticks with tolerance
        public void raiseSlides(int targetTicks, double power) {
            int targetPosition = Math.min(targetTicks, MAX_TICKS);  // Cap target at MAX_TICKS
            vertL.setTargetPosition(targetPosition);
            vertR.setTargetPosition(targetPosition);
            vertL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            vertR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            // Start moving with initial power
            vertL.setPower(power);
            vertR.setPower(power);
            isAtTarget = false;  // Reset target flag

            while (opModeIsActive() && !isAtTarget) {
                int currentPosition = vertL.getCurrentPosition();  // Only use vertL encoder

                if (Math.abs(currentPosition - targetPosition) <= TOLERANCE) {
                    isAtTarget = true;  // Mark as at target
                    telemetry.addData("Slides", "Reached target, stopping");
                    telemetry.update();
                    stopSlides();  // Stop the slides when within tolerance
                } else if (Math.abs(currentPosition - targetPosition) <= TOLERANCE * 2) {
                    // If close to the target but not within the tolerance, reduce power to prevent overshoot
                    vertL.setPower(LOW_POWER);
                    vertR.setPower(LOW_POWER);
                    telemetry.addData("Slides", "Approaching target, reducing power");
                } else {
                    telemetry.addData("Raising Slides", "Position: %d, Target: %d", currentPosition, targetPosition);
                }
                telemetry.update();
            }
        }

        // Method to lower the slides using target ticks with tolerance
        public void lowerSlides(int targetTicks, double power) {
            int targetPosition = Math.max(-targetTicks, 0);  // Cap target to avoid going below zero
            vertL.setTargetPosition(targetPosition);
            vertR.setTargetPosition(targetPosition);
            vertL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            vertR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            // Start moving with initial power
            vertL.setPower(-power);
            vertR.setPower(-power);
            isAtTarget = false;  // Reset target flag

            while (opModeIsActive() && !isAtTarget) {
                int currentPosition = vertL.getCurrentPosition();  // Only use vertL encoder

                if (Math.abs(currentPosition - targetPosition) <= TOLERANCE) {
                    isAtTarget = true;  // Mark as at target
                    telemetry.addData("Slides", "Reached target, stopping");
                    telemetry.update();
                    stopSlides();  // Stop the slides when within tolerance
                } else if (Math.abs(currentPosition - targetPosition) <= TOLERANCE * 2) {
                    // If close to the target but not within the tolerance, reduce power to prevent overshoot
                    vertL.setPower(-LOW_POWER);
                    vertR.setPower(-LOW_POWER);
                    telemetry.addData("Slides", "Approaching target, reducing power");
                } else {
                    telemetry.addData("Lowering Slides", "Position: %d, Target: %d", currentPosition, targetPosition);
                }
                telemetry.update();
            }
        }

        // Method to stop the slides and set motors to hold position
        public void stopSlides() {
            vertL.setPower(0);
            vertR.setPower(0);
            vertL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);  // Force the motor to hold its position
            vertR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);  // Same for vertR
            telemetry.addData("Slides", "Stopped and holding position");
            telemetry.update();
        }
    }

    // SlideLiftAction class to wrap slide lift actions
    public class SlideLiftAction implements Action {
        private SlideLift slideLift;
        private int targetTicks;
        private double power;
        private boolean isRaise;

        public SlideLiftAction(SlideLift slideLift, int targetTicks, double power, boolean isRaise) {
            this.slideLift = slideLift;
            this.targetTicks = targetTicks;
            this.power = power;
            this.isRaise = isRaise;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (isRaise) {
                slideLift.raiseSlides(targetTicks, power);  // Raise the slides
            } else {
                slideLift.lowerSlides(targetTicks, power);  // Lower the slides
            }
            return true;  // Return true to indicate completion
        }
    }

    // Custom WaitAction class for timed waits
    public class WaitAction implements Action {
        private ElapsedTime timer = new ElapsedTime();
        private double seconds;

        public WaitAction(double seconds) {
            this.seconds = seconds;
            timer.reset();  // Start the timer
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            return timer.seconds() >= seconds;  // Returns true when wait time has passed
        }
    }

    @Override
    public void runOpMode() {
        // Initialize robot hardware and systems
        Pose2d initialPose = new Pose2d(-36, -60, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        slideLift = new SlideLift(hardwareMap);

        // Reset slide lift position to zero at the start
        slideLift.raiseSlides(0, 0);  // Ensure slides are at 0 ticks before starting

        // Initialization loop that runs while the robot is waiting to start
        while (!isStarted() && !isStopRequested()) {
            double parkLocation = 1;
            telemetry.addData("Status", "Running INIT:LOOP--WAITING FOR START");
            if (parkLocation == 1) {
                telemetry.addData("Park Location 1:", "Level 1 Ascent");
            } else if (parkLocation == 2) {
                telemetry.addData("Park Location 2:", "OBSERVATION: CORNER");
            } else {
                telemetry.addData("Park Location 3:", "OBSERVATION: EDGE");
            }
            telemetry.update();
        }

        waitForStart();

        if (opModeIsActive()) {
            // Start the autonomous sequence
            Actions.runBlocking(new SequentialAction(
                    // Robot movement action
                    drive.actionBuilder(initialPose)
                            .strafeTo(new Vector2d(-58, -52))  // Move to first position
                            .turn(Math.toRadians(55))          // Turn to face the target
                            .build(),

                    // Raise slides using custom SlideLiftAction with tolerance
                    new SlideLiftAction(slideLift, 3000, 1.0, true),

                    // Wait for 1 second using custom WaitAction
                    new WaitAction(1.0),

                    // Lower slides using custom SlideLiftAction with tolerance
                    new SlideLiftAction(slideLift, 0, 1.0, false),

                    // Continue with the autonomous path
                    drive.actionBuilder(new Pose2d(-58, -52, Math.toRadians(55))) // Using the last known position
                            .strafeTo(new Vector2d(-40, -45)) // Move to the next position
                            .build()
            ));
        }
    }
}
