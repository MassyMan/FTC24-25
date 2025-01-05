package org.firstinspires.ftc.teamcode.Autonomous.VerticalSlides;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.Range;

public class SlideLiftAction implements Action {
    private org.firstinspires.ftc.teamcode.Autonomous.VerticalSlides.liftSlides liftSlides;
    private double targetTicks;

    public SlideLiftAction(liftSlides liftSlides, double targetTicks) {
        this.liftSlides = liftSlides;
        this.targetTicks = Range.clip(targetTicks, 0, 1960);
    }

    @Override
    public boolean run(TelemetryPacket packet) {
        liftSlides.moveSlides(targetTicks);
        boolean isAtTarget = liftSlides.isAtTarget();
        if (isAtTarget) {
            liftSlides.stopSlides();
        }

        return !isAtTarget;
    }
}
