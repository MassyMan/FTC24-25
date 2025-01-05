package org.firstinspires.ftc.teamcode.Autonomous.VerticalSlides;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class liftSlides {
    private DcMotorEx vertL;
    private DcMotorEx vertR;

    private double targetPosition = 0;

    public static double kP = 0.08;
    public static double kF = 0.2;
    public static final int THRESHOLD = 80;
    private static final double HOLD_POWER = 0.15;
    private static final double MIN_DOWN_POWER = -0.90;

    public liftSlides(HardwareMap hardwareMap) {
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
        targetPosition = targetTicks;
        int currentPosition = vertL.getCurrentPosition();
        double error = targetPosition - currentPosition;
        double power = kP * error + kF;
        power = Range.clip(power, MIN_DOWN_POWER, 1.0);

        if (power < 0) {
            power = Math.max(power, MIN_DOWN_POWER);
        }

        if ((currentPosition <= 10) && (power < 0) && targetPosition == 0) {
            stopSlides();
            power = 0;
            if (currentPosition < 0){
                vertL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            }
        }

        vertL.setPower(power);
        vertR.setPower(power);

        if (Math.abs(error) <= THRESHOLD && targetPosition != 0) {
            vertL.setPower(HOLD_POWER);
            vertR.setPower(HOLD_POWER);
        }
    }

    public void stopSlides() {
        if (targetPosition != 0) {
            vertL.setPower(HOLD_POWER);
            vertR.setPower(HOLD_POWER);
        } else {
            vertL.setPower(0);
            vertR.setPower(0);
        }
    }

    public boolean isAtTarget() {
        return Math.abs(targetPosition - vertL.getCurrentPosition()) <= THRESHOLD;
    }
}
