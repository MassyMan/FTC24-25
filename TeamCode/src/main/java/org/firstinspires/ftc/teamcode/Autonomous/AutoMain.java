package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer;

@Autonomous(group = "a")
public class AutoMain extends LinearOpMode {
    private V4Bar v4Bar;
    private Extendo extendo;
    private VerticalSlides verticalSlides;
    private RobotSetup robotSetup;

    private Pose2d startPose; // START LOCATION
    private Pose2d parkPose; // PARK LOCATION
    private Pose2d currentPose; // CURRENT POSE

    ThreeDeadWheelLocalizer localizer;
    double pathSpeed = 0;

    public void updatePose(){
        Twist2dDual<Time> twist = localizer.update();
        currentPose = currentPose.plus(twist.value());
    }

    private TrajectoryActionBuilder pepePath(MecanumDrive drive, Pose2d herePose, Vector2d target, double heading, double speed) {
        return drive.actionBuilder(herePose)
                .strafeToLinearHeading(
                        target,
                        heading,
                        new TranslationalVelConstraint(speed),
                        new ProfileAccelConstraint(-speed, speed)
                );

    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("AUTO TYPE:", robotSetup.autoType);
        telemetry.update();

        // AUTO INITIALIZER
        switch (robotSetup.autoType){

            case "SAMPLE": // SAMPLE AUTO SETUP

                switch (robotSetup.preloadType){ // START POSE SWITCH [PRELOAD TYPE]
                    case "SAMPLE":
                        startPose = new Pose2d(0, 0, Math.toRadians(0)); // TODO: INPUT
                        break;

                    case "SPECIMEN":
                        startPose = new Pose2d(1, 0, Math.toRadians(0)); // TODO: INPUT
                        break;

                    default:
                        telemetry.addLine("WARNING: NO PRELOAD TYPE DECLARED! USING DEFAULT PRELOAD STARTPOSE.");
                        telemetry.update();
                        startPose = new Pose2d(0, 0, Math.toRadians(0)); // TODO: INPUT
                        break;
                }

                switch (robotSetup.parkLocation){ // PARKING LOCATION SWITCH

                    case "TOUCHING_BAR":
                        parkPose = new Pose2d(0, 0, Math.toRadians(0)); // TODO: INPUT
                        break;

                    default:
                        break;

                }
                break;

            case "SPECIMEN": // SPECIMEN AUTO SETUP

                switch (robotSetup.parkLocation) {
                    case "FLOAT":
                        parkPose = new Pose2d(0, 0, Math.toRadians(0)); // TODO: INPUT
                        break;

                    case "PERPENDICULAR":
                        parkPose = new Pose2d(1, 0, Math.toRadians(0)); // TODO: INPUT
                        break;

                    case "SPECIMEN":
                        parkPose = new Pose2d(121, 0, Math.toRadians(0)); // TODO: INPUT
                        break;

                    default:
                        telemetry.addLine("[SPECIMEN AUTO] WARNING! NO PARK LOCATION FOUND!");
                        parkPose = new Pose2d(0, 0, Math.toRadians(0)); // TODO: INPUT, FLOAT
                        break;
                }
                break;


        }
        // TODO: ======================================================================================================================================


        /*
                    90
                     ^
                     |
           < 180  ------------ > 0
                     |
                     V
                    270

        */


        // MAIN AUTONOMOUS SEQUENCE
        if (opModeIsActive()) {
            // SETUP
            MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
            currentPose = startPose;

            if ("SPECIMEN".equals(robotSetup.autoType)) { // SPECIMEN AUTO SEQUENCE
                Actions.runBlocking(

                        pepePath(drive, currentPose, new Vector2d(20, 20), Math.toRadians(180), 80)
                                .build()
                );
                updatePose();



            } else if ("SAMPLE".equals(robotSetup.autoType)) {

            }



        }


    }
}

