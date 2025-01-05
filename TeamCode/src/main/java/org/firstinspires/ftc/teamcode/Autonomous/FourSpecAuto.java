
package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Autonomous.VerticalSlides.SlideLiftAction;
import org.firstinspires.ftc.teamcode.Autonomous.VerticalSlides.liftSlides;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.NUSpecs;
import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer;

@Autonomous(group = "a")
public class FourSpecAuto extends LinearOpMode {
    private V4Bar v4Bar;
    private Servo v4BarServo;
    private CRServo intakeL, intakeR;
    private Extendo extendo;
    private RobotSetup robotSetup;
    private liftSlides verticalSlides;

    private Pose2d startPose; // START LOCATION
    private Pose2d parkPose; // PARK LOCATION

    double pathSpeed = 0;

    @Override
    public void runOpMode() throws InterruptedException {
     //   telemetry.addData("AUTO TYPE:", robotSetup.autoType);
      //  telemetry.update();

        v4BarServo = hardwareMap.get(Servo.class, "v4Bar");
        intakeL = hardwareMap.get(CRServo.class, "intakeL");
        intakeR = hardwareMap.get(CRServo.class, "intakeR");

        /*
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
                startPose = new Pose2d(10, -60, Math.toRadians(0)); // SPECIMEN AUTO START POSE
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
        telemetry.update();


         */
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
        waitForStart();
        // MAIN AUTONOMOUS SEQUENCE
        if (opModeIsActive()) {
            // SETUP
            MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
            startPose = new Pose2d(10, -60, Math.toRadians(90)); // ROBOT STARTING POSE


            SlideLiftAction slidesSpecimen = new SlideLiftAction(verticalSlides, 750);
            SlideLiftAction slidesGround = new SlideLiftAction(verticalSlides, 0);
            V4Bar V4BarDeposit = new V4Bar(v4BarServo, 0.17);

            Actions.runBlocking(drive.actionBuilder(startPose)
                    .afterTime(0, V4BarDeposit)
                    .afterTime(0, slidesSpecimen)
                    .afterTime(0.8, slidesSpecimen)
                    .strafeToLinearHeading(new Vector2d(4, -32), Math.toRadians(90),
                            new TranslationalVelConstraint(80),
                            new ProfileAccelConstraint(-80, 80))
                    .strafeToLinearHeading(new Vector2d(40, -38), Math.toRadians(50),
                            new TranslationalVelConstraint(80),
                            new ProfileAccelConstraint(-80, 80))
                    .build());







        }


    }
}
