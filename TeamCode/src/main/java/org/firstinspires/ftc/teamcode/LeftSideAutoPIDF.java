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
    private CRServo intake2;

    // PIDF control variables
    public static double kP = 0.005;
    public static double kF = 0.2;
    public static final int THRESHOLD = 80;
    private static final int MAX_TICKS = 3900;
    private static final double HOLD_POWER = 0.1;
    private static final double MIN_DOWN_POWER = -0.2;

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

            // Check if we are within the threshold
            if (Math.abs(error) <= THRESHOLD) {
                applyHoldPower(); // Maintain the current position with hold power
                telemetry.addData("Slide Lift", "Holding at Position: %d", currentPosition);
                return; // Exit the method, stopping further actions
            }

            // Calculate power based on error
            double power = kP * error + kF;
            power = Range.clip(power, -1.0, 1.0);

            // Prevent going below the minimum
            if (power < 0 && currentPosition <= 0) {
                power = 0; // Prevent going below the minimum
            } else if (power < 0) {
                power = Math.max(power, MIN_DOWN_POWER); // Ensure it goes down faster
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


        public void applyHoldPower() {
            vertL.setPower(HOLD_POWER);
            vertR.setPower(HOLD_POWER);
            telemetry.addData("Slide Lift", "Holding Power Applied: %.2f", HOLD_POWER);
            telemetry.update();
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
            return !slideLift.isAtTarget();
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
            return false; // Returning false to indicate the action is instantaneous.
        }
    }

    @Override
    public void runOpMode() {
        Pose2d startPose = new Pose2d(-10, -60, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        slideLift = new SlideLift(hardwareMap);
        v4Bar = hardwareMap.get(Servo.class, "v4Bar");
        intake = hardwareMap.get(CRServo.class, "intake");
        intake2 = hardwareMap.get(CRServo.class, "intake2");

        waitForStart();

        if (opModeIsActive()) {
            SlideLiftAction liftAction = new SlideLiftAction(slideLift, 1700);
            V4BarAction v4BarAction = new V4BarAction(v4Bar, 0.6); // Set to desired position

            // TODO: Intake while lowering slides from 1600 -> 0 to hang specimen on bar, 11/3 deadline for POC hang

            Actions.runBlocking(drive.actionBuilder(startPose)
                    .afterDisp(0, liftAction)
                    .afterDisp(0, new V4BarAction(v4Bar, 0.5)) // Move the v4Bar after lifting
                    .strafeTo(new Vector2d(-10, -37))
                    .waitSeconds(2)
                    .build());

            SlideLiftAction lowerAction = new SlideLiftAction(slideLift, 150);
            Actions.runBlocking(drive.actionBuilder(new Pose2d(-10, -37, Math.toRadians(90)))
                    .afterDisp(0, lowerAction)
                    .afterDisp(10, new V4BarAction(v4Bar, 0.94))
                    .strafeTo(new Vector2d(-10, -45))
                    .strafeTo(new Vector2d(-50, -45))
                    .build());
        }
    }
}
