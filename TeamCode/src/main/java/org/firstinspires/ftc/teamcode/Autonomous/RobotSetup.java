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
        waitForStart();
    }
}
