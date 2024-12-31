package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@Autonomous(name = "Diddy Party Auto")
public class DiddyPartyAuto extends LinearOpMode {

    ThreeDeadWheelLocalizer localizer;
    Pose2d pose = new Pose2d(0, 0, 0);
    ElapsedTime elapsedTime;

    // Initialize Variables
    double targetX = 0;
    double targetY = 0;
    double targetT = 0;
    double currentX = 0;
    double currentY = 0;
    double currentT = 0;
    double errorX = 0;
    double errorY = 0;
    double errorT = 0;
    double lastErrorX = 0;
    double lastErrorY = 0;
    double lastErrorT = 0;
    // x/y/t/ for drivetrain math [NOT DRIVETRAIN POSITION]
    double x = 0;
    double y = 0;
    double t = 0;
    double xKP = 0.0001;
    double xKD = 0.0001;
    double yKP = 0.0001;
    double yKD = 0.0001;
    double tKP = 0.0001;
    double tKD = 0.0001;
    double x_rotated = 0;
    double y_rotated = 0;

    // Initialize Motors
    DcMotor leftFront, leftBack, rightBack, rightFront;
    DcMotor vertL, vertR;
    DcMotor wormL, wormR;

    // Initialize Servos
    DcMotor extendoEncoder; // REV Robotics through-bore encoder attached to horizontal extension
    CRServo slidL, slidR, intakeL, intakeR;
    Servo V4Bar;

    // Initialize Scheduler
    private ActionScheduler scheduler;

    // ===============================================================================

    public void calculatePID(){
        // Update current position variables
        currentX = pose.position.x;
        currentY = pose.position.y;
        currentT = pose.heading.toDouble();
        // Calculate error
        errorX = currentX - targetX;
        errorY = currentY - targetY;
        errorT = currentT - Math.toRadians(targetT);

        // Run PID algorithm based on error
        x = (errorX * xKP) + (((errorX - lastErrorX) / elapsedTime.seconds()) * xKD);
        y = (errorY * yKP) + (((errorY - lastErrorY) / elapsedTime.seconds()) * yKD);
        t = (errorT * tKP) + (((errorT - lastErrorT) / elapsedTime.seconds()) * tKD);
        // Reset Timer
        lastErrorX = errorX;
        lastErrorY = errorY;
        lastErrorT = errorT;
        elapsedTime.reset();
    }

    public void pidToPoint(double xPos, double yPos, double heading, double moveSpeed, double tolerance, long waitTime) {
        // Register target positions
        targetX = xPos;
        targetY = yPos;
        targetT = heading; // in degrees

        // If the error is larger than the tolerance
        if ((Math.sqrt(Math.pow(currentX - targetX, 2) + Math.pow(currentY - targetY, 2))) > tolerance) {
            // Run PID algorithm
            calculatePID();

            // Rotate powers to be field centric
            x_rotated = x * Math.cos(currentT) - y * Math.sin(currentT);
            y_rotated = x * Math.sin(currentT) + y * Math.cos(currentT);

            // Apply motor powers
            leftFront.setPower((x_rotated + y_rotated + t) * (moveSpeed / 100));
            leftBack.setPower((x_rotated - y_rotated + t) * (moveSpeed / 100));
            rightFront.setPower((x_rotated - y_rotated - t) * (moveSpeed / 100));
            rightBack.setPower((x_rotated + y_rotated - t) * (moveSpeed / 100));

        }
    }

    public void moveSlides(double targetPos, double moveSpeed, boolean holdPosition){

    }

    public void moveExtendo(double targetPos, double moveSpeed){

    }

    private void slidesSpecimen() {
        telemetry.addData("Running intakeSample . . .", "");
        telemetry.update();
        moveSlides(500, 1, true);
    }

    // ===============================================================================

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        elapsedTime = new ElapsedTime();
        // Initialize Drivetrain
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);

        // Initialize Vertical Slides
        vertL = hardwareMap.get(DcMotor.class, "vertL");
        vertR = hardwareMap.get(DcMotor.class, "vertR");

        // Initialize Horizontal Slides
        extendoEncoder = hardwareMap.get(DcMotorEx.class, "vertR"); // Rev through-bore encoder plugged into vertR
        extendoEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendoEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slidL = hardwareMap.get(CRServo.class, "slidL");
        slidR = hardwareMap.get(CRServo.class, "slidR");
        V4Bar = hardwareMap.get(Servo.class, "v4Bar");
        intakeL = hardwareMap.get(CRServo.class, "intakeL");
        intakeR = hardwareMap.get(CRServo.class, "intakeR");

        // Initialize Hang
        wormL = hardwareMap.get(DcMotor.class, "wormL");
        wormR = hardwareMap.get(DcMotor.class, "wormR");

        // Initialize RR1.0 Localizer
        localizer = new ThreeDeadWheelLocalizer(hardwareMap, 0.0019547157517511);

        // Wait until program is started
        waitForStart();

        // STARTING POSITION

        // Initialize sequential actions
        scheduler = new ActionScheduler();

        scheduler.schedule(1, this::slidesSpecimen);

        // Loop
        while (opModeIsActive()) {
            Twist2dDual<Time> twist = localizer.update();
            pose = pose.plus(twist.value());

            telemetry.addData("Running DiddyPartyAuto", "");
            telemetry.addData("Current X", currentX);
            telemetry.addData("Current Y", currentY);
            telemetry.addData("Current T", currentT);
            telemetry.addData("Target X", targetX);
            telemetry.addData("Target Y", targetY);
            telemetry.addData("Target T", targetT);
            telemetry.addData("x", x);
            telemetry.addData("y", y);
            telemetry.addData("t", t);
            telemetry.addData("Version ", "110");
            telemetry.update();


          //  scheduler.runScheduled();

            pidToPoint(10, 10, 0, 60, 2, 0);




        }


    }
}
