package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class pidfLauncher extends OpMode {

    private DcMotorEx intakeWheels;
    private DcMotorEx launcherLeft;
    private Servo kicker;

    public double highVelocity = 1700;
    public double lowVelocity = 1000;
    public double curVelocityTarget = highVelocity;

    double P = 0;
    double F = 0;

    double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001, 0.0001};

    int stepIndex = 1;


    public void runopmode() {

    }

    public void init() {

        launcherLeft = hardwareMap.get(DcMotorEx.class, "LL");
        launcherLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        launcherLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    public void loop() {
        if (gamepad1.yWasPressed()) {
            if (curVelocityTarget == highVelocity) {
                curVelocityTarget = lowVelocity;
            } else curVelocityTarget = highVelocity;
        }

        if(gamepad1.bWasPressed()){
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if(gamepad1.dpadLeftWasPressed()){
            F += stepSizes[stepIndex];
        }

        if (gamepad1.dpadRightWasPressed()){
            F -= stepSizes[stepIndex];
        }

        if (gamepad1.dpadUpWasPressed()){
            P += stepSizes[stepIndex];
        }

        if (gamepad1.dpadDownWasPressed()){
            P -= stepSizes[stepIndex];
        }

        // set new Pidf coefficents
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        launcherLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        //set motor velocity


        launcherLeft.setVelocity(curVelocityTarget);

        double curVelocity = launcherLeft.getVelocity();
        double error = curVelocityTarget - curVelocity;

        telemetry.addData("Target Velocity", curVelocityTarget);
        telemetry.addData("Current Velocity", "%.2f", curVelocity);
        telemetry.addData("Error", "&.2f", error);
        telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", P);
        telemetry.addData("Tuning F", "%.4f (D-Pad F/R", F);
        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);

    }
}