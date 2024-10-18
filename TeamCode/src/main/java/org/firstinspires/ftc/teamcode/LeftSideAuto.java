package org.firstinspires.ftc.teamcode;

// RR-specific imports
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import java.util.Vector;

@Config
@Autonomous(name = "LEFT 1+3", group = "Autonomous")
public class LeftSideAuto extends LinearOpMode {

    private SlideLift slideLift;

    public class SlideLift {
        private DcMotorEx vertL;
        private DcMotorEx vertR;

        public SlideLift(HardwareMap hardwareMap) {
            vertL = hardwareMap.get(DcMotorEx.class, "vertL");
            vertR = hardwareMap.get(DcMotorEx.class, "vertR");

            // Set zero power behavior to brake
            vertL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            vertR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

            // Set motor directions to spin in opposite directions
            vertL.setDirection(DcMotorSimple.Direction.FORWARD);
            vertR.setDirection(DcMotorSimple.Direction.REVERSE);

            // Reset encoder and set mode to run using encoder
            vertL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            vertL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }

        // Method to raise the slides using encoder ticks
        public void raiseSlides(int targetTicks, double power) {
            vertL.setTargetPosition(targetTicks);
            vertL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            vertL.setPower(power);

            while (vertL.isBusy()) {
                // Wait until motor reaches the target
            }

            stopSlides();
            vertL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }

        // Method to lower the slides using encoder ticks
        public void lowerSlides(int targetTicks, double power) {
            raiseSlides(-targetTicks, power);
        }

        // Method to stop the slides
        public void stopSlides() {
            vertL.setPower(0);
            vertR.setPower(0);
        }
    }

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-36, -60, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        slideLift = new SlideLift(hardwareMap);

        // Initialization loop that runs while the robot is waiting to start
        while (!isStarted() && !isStopRequested()) {
            // Add your initialization code here
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
            // Autonomous actions go here

            /* Action Runslides;
                slideLift.raiseSlides(1400,0.8);
                slideLift.lowerSlides(1400,0.3);
               */

            Actions.runBlocking(new SequentialAction(
                    // Add your autonomous actions here
                    drive.actionBuilder(initialPose)
                            .strafeTo(new Vector2d(-58, -52))
                            .turn(Math.toRadians(55))
                            .waitSeconds(1)
                          //  Runslides
                            .strafeTo(new Vector2d(-39, -54))
                            .strafeTo(new Vector2d(-39, -22))
                            .waitSeconds(2)
                            .strafeTo(new Vector2d(-36, -60))
                            .build()
            ));

            // Example: Raise slides
            // slideLift.raiseSlides(1000, 1.0); // Adjust ticks and power as needed

            // Example: Lower slides
            // slideLift.lowerSlides(1000, 1.0); // Adjust ticks and power as needed
        }
    }
}
