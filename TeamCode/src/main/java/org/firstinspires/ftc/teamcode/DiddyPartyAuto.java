package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Twist2dDual;
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

@Autonomous(name = "Diddy Party Auto")
public class DiddyPartyAuto extends LinearOpMode {

    ThreeDeadWheelLocalizer localizer;
    Pose2d pose;

    @Override
    public void runOpMode() throws InterruptedException {

        // initialize localizer
        localizer = new ThreeDeadWheelLocalizer(hardwareMap, 0.0019547157517511);

        // initialize variables
        double targetX = 0;
        double targetY = 0;
        double targetT = 0;
        double errorX = 0;
        double errorY = 0;
        double errorT = 0;
        double lastErrorX = 0;
        double lastErrorY = 0;
        double lastErrorT = 0;
        double xInput = 0;
        double yInput = 0;
        double tInput = 0;
        double xKP = 0;
        double xKD = 0;
        double yKP = 0;
        double yKD = 0;
        double tKP = 0;
        double tKD = 0;

        ElapsedTime elapsedTime;

        // Wait until program is started
        waitForStart();

        // STARTING POSITION
        Pose2d pose = new Pose2d(0, 0, Math.toRadians(90));

        elapsedTime = new ElapsedTime();

        // Loop
        while (opModeIsActive()) {

            double currentX = pose.position.x;
            double currentY = pose.position.y;
            double currentT = pose.heading.toDouble();

            Twist2dDual<Time> twist = localizer.update();
            pose = pose.plus(twist.value());

            telemetry.addData("Pose X", currentX);
            telemetry.addData("Pose Y", currentY);
            telemetry.addData("Heading", currentT);
            telemetry.addData("Loop Time", elapsedTime.seconds());
            telemetry.update();

            elapsedTime.reset();
        }


    }
}
