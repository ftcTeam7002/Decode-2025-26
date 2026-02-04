/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "Odo test2", group = "Robot")
public class Algorithm_2 extends LinearOpMode {

    private IMU imu;

    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;
    public DcMotor odometerLeft;
    public DcMotor odometerAux;

    private DcMotorEx intakeWheels;
    private DcMotorEx launcherLeft;
    private Servo kicker;

    static final double FORWARD_SPEED = 0.6;

    // Your odometry constant
    public static final double TICKS_PER_INCH = 1058.34;

    @Override
    public void runOpMode() {

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                );
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        frontLeftDrive  = hardwareMap.get(DcMotor.class, "FL");
        frontRightDrive = hardwareMap.get(DcMotor.class, "FR");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "BL");
        backRightDrive  = hardwareMap.get(DcMotor.class, "BR");
        intakeWheels = hardwareMap.get(DcMotorEx.class, "IW");
        kicker = hardwareMap.get(Servo.class, "kicker");
        launcherLeft = hardwareMap.get(DcMotorEx.class, "LL");
        odometerLeft = backLeftDrive;
        odometerAux  = frontRightDrive;

        intakeWheels.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Initialized — waiting for start");
        telemetry.update();

        waitForStart();

        imu.resetYaw();


        driveToTicks((int)(15 * TICKS_PER_INCH));



        telemetry.addLine("COMPLETE");
        telemetry.update();
        sleep(1000);
    }


    private void driveToTicks(int targetTicks) {

        // Reset odometry encoders
        odometerLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odometerAux.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odometerLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odometerAux.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // drive
        frontLeftDrive.setPower(FORWARD_SPEED);
        frontRightDrive.setPower(FORWARD_SPEED);
        backLeftDrive.setPower(FORWARD_SPEED);
        backRightDrive.setPower(FORWARD_SPEED);
        sleep(700);
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);

        while (opModeIsActive()
                && Math.abs(odometerLeft.getCurrentPosition()) < targetTicks
                && Math.abs(odometerAux.getCurrentPosition()) < targetTicks) {

            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
            double yaw = angles.getYaw(AngleUnit.DEGREES);

            telemetry.addData("Yaw", yaw);
            telemetry.addData("Left Odo", odometerLeft.getCurrentPosition());
            telemetry.addData("Aux Odo", odometerAux.getCurrentPosition());
            telemetry.addData("Target", targetTicks);
            telemetry.update();
        }

        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);

        sleep(200);
    }
}
