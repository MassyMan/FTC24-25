package org.firstinspires.ftc.teamcode.Autonomous;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;

public class V4Bar implements Action {
    private Servo v4Bar;
    private double position;


    public V4Bar(Servo v4Bar, double position) {
        this.v4Bar = v4Bar;
        this.position = position;
    }

    @Override
    public boolean run(TelemetryPacket packet) {
        v4Bar.setPosition(position);
        return false;
    }
}
