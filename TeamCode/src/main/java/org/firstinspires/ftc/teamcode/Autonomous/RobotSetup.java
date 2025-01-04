package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@TeleOp
public class RobotSetup extends LinearOpMode {

    public boolean advance = false, specimenAuto = false, sampleAuto = false, parkAuto = false;
    public double startDelay = 0;
    public String autoType = null;
    public String parkLocation = null;
    public String sample5th = null;
    public String preloadType = null;

    @Override
    public void runOpMode() throws InterruptedException {

        // Determine AUTONOMOUS Type
        while (!advance && opModeInInit()) {
            telemetry.addData("Press INIT To Begin RobotSetup.java", "");
            telemetry.addData("USE GAMEPAD 1 [BLUE] To Select Autonomous Type", "");
            telemetry.addLine();
            telemetry.addData("LEFT ---> SAMPLE AUTO", "");
            telemetry.addData("RIGHT --> SPECIMEN AUTO", "");
            telemetry.addData("UP -----> PARK AUTO", "");
            telemetry.update();

            if (gamepad1.dpad_left) {
                autoType = "SAMPLE";
                advance = true;
            }
            if (gamepad1.dpad_right) {
                autoType = "SPECIMEN";
                advance = true;
            }
            if (gamepad1.dpad_up) {
                autoType = "PARK";
                advance = true;
            }
        }



        if (autoType.equals("SPECIMEN")){ // ============================================ SPECIMEN AUTO
            // Reset variables for next selection
            advance = false;
            telemetry.clearAll();

            // Determine Park Orientation
            while (!advance && opModeInInit()) {
                telemetry.addLine("SELECT DESIRED PARK ORIENTATION");
                telemetry.addLine("LEFT ---> FLOAT [DIAGONAL]");
                telemetry.addLine("RIGHT --> PERPENDICULAR [FACING OTHER ALLIANCE]");
                telemetry.addLine("UP -----> FACING SPECIMEN [RIGHT WALL]");
                telemetry.addLine("DOWN ---> NO PARK");
                telemetry.update();

                if (gamepad1.dpad_left){
                    parkLocation = "FLOAT";
                    advance = true;
                }
                if (gamepad1.dpad_right){
                    parkLocation = "PERPENDICULAR";
                    advance = true;
                }
                if (gamepad1.dpad_up){
                    parkLocation = "SPECIMEN";
                    advance = true;
                }
                if (gamepad1.dpad_down){
                    advance = true;
                }
            }

            // Reset variables for next selection
            advance = false;
            telemetry.clearAll();

        } else if (autoType.equals("SAMPLE")){ // ==================================== SAMPLE AUTO
            // Reset variables for next selection

            while (!advance && opModeInInit()) {
                telemetry.addLine("[SAMPLE AUTO] INPUT PRELOAD TYPE");
                telemetry.addLine("LEFT ---> SAMPLE");
                telemetry.addLine("RIGHT --> SPECIMEN");
                telemetry.addLine("UP -----> NONE");
                telemetry.addLine("DOWN ---> NONE");

                telemetry.update();

                if (gamepad1.dpad_left){
                    preloadType = "SAMPLE";
                    advance = true;
                }
                if (gamepad1.dpad_right){
                    preloadType = "SPECIMEN";
                    advance = true;
                }
                if (gamepad1.dpad_up){
                    preloadType = "NONE";
                    advance = true;
                }
                if (gamepad1.dpad_down){
                    preloadType = "NONE";
                    advance = true;
                }
            }

            advance = false;
            telemetry.clearAll();

            while (!advance && opModeInInit()) {
                telemetry.addLine("GRAB 5th YELLOW SAMPLE?");
                telemetry.addLine("LEFT ---> TRUE: DRIVE IN FRONT OF IDLE ROBOT");
                telemetry.addLine("RIGHT --> FALSE");
                telemetry.addLine("UP -----> TRUE: GRAB FROM OTHER ROBOT [THEIR PRELOAD]");
                telemetry.addLine("DOWN ---> TRUE: DRIVE THROUGH BOTTOM ROW");
                telemetry.update();

                if (gamepad1.dpad_left){
                    sample5th = "DRIVETOP";
                    advance = true;
                }
                if (gamepad1.dpad_right){
                    sample5th = "FALSE";
                    advance = true;
                }
                if (gamepad1.dpad_up){
                    sample5th = "FROMOTHER";
                    advance = true;
                }
                if (gamepad1.dpad_down){
                    sample5th = "DRIVEBOTTOM";
                    advance = true;
                }
            }

            while (!advance && opModeInInit()) {
                telemetry.addLine("[SAMPLE AUTO] INPUT PARK LOCATION");
                telemetry.addLine("LEFT ---> TOUCHING BAR");
                telemetry.addLine("RIGHT --> NONE");
                telemetry.addLine("UP -----> NONE");
                telemetry.addLine("DOWN ---> NONE");

                telemetry.update();

                if (gamepad1.dpad_left){
                    preloadType = "TOUCHING_BAR";
                    advance = true;
                }
                if (gamepad1.dpad_right){
                    preloadType = "NONE";
                    advance = true;
                }
                if (gamepad1.dpad_up){
                    preloadType = "NONE";
                    advance = true;
                }
                if (gamepad1.dpad_down){
                    preloadType = "NONE";
                    advance = true;
                }
            }

            advance = false;
            telemetry.clearAll();

        } else if (autoType.equals("PARK")){
            // TODO: ADD PARK AUTO CODE
        }




    }
}
