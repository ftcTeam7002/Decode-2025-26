package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "Odo3", group = "Robot")
public class Odometry_3 extends LinearOpMode {

    // Drive
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    public DcMotor odometerRight;
    public DcMotor odometerAux;
    // Mechanisms
    private DcMotorEx intakeWheels;
    private DcMotorEx launcherLeft;
    private Servo kicker;

    // Sensors
    private IMU imu;

    // Constants
    public static final double inches = 336.9;
    public static final double strafeInches = 355;

    @Override
    public void runOpMode() {
        //----------------IMU----------------------
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                );
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // -------------------- Hardware Map --------------------
        imu = hardwareMap.get(IMU.class, "imu");

        frontLeftDrive = hardwareMap.get(DcMotor.class, "FL");
        frontRightDrive = hardwareMap.get(DcMotor.class, "FR");
        backLeftDrive = hardwareMap.get(DcMotor.class, "BL");
        backRightDrive = hardwareMap.get(DcMotor.class, "BR");

        intakeWheels = hardwareMap.get(DcMotorEx.class, "IW");
        launcherLeft = hardwareMap.get(DcMotorEx.class, "LL");
        kicker = hardwareMap.get(Servo.class, "kicker");


        odometerRight = frontRightDrive;
        frontRightDrive.getCurrentPosition();
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        odometerAux = frontLeftDrive;
        frontLeftDrive.getCurrentPosition();
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // -------------------- Motor Direction --------------------
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        intakeWheels.setDirection(DcMotor.Direction.REVERSE);

        //--------------------BRAKE---------------------------
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//                              Reset Mechanisms
        resetEncoders();
        kicker.setPosition(0);
        waitForStart();
        imu.resetYaw();


        // -----------------------CODE SEQUENCE-----------------------------------

//        forward(0.3,(int)(23*inches));
//        sleep(50);
//        resetEncoders();
//        backwards(0.3, (int)(23*-inches));
//        sleep(50);
//        resetEncoders();
        strafe(0.3, (int) (5 * strafeInches));
        sleep(50);
        resetEncoders();
//      launch();


        telemetry.update();
        while (opModeIsActive()) {
            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
            double yaw = angles.getYaw(AngleUnit.DEGREES);

            telemetry.addData("Yaw", yaw);
            telemetry.addData("Left Odo", odometerRight.getCurrentPosition());
            telemetry.addData("Aux Odo", odometerAux.getCurrentPosition());
            telemetry.update();
        }
    }

    //---------------------------FORWARD------------------------------------------
    private void forward(double desiredPower, int ticks) {

        frontLeftDrive.setPower(desiredPower);
        frontRightDrive.setPower(desiredPower);
        backLeftDrive.setPower(desiredPower);
        backRightDrive.setPower(desiredPower);

        while (odometerRight.getCurrentPosition() < ticks) {
            try {
                sleep(25);
            } catch (Exception e) {
                throw new RuntimeException(e);
            }
        }
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);

    }

    // --------------------BACKWARDS------------------------------------
    private void backwards(double desiredPower, int ticks) {

        frontLeftDrive.setPower(-desiredPower);
        frontRightDrive.setPower(-desiredPower);
        backLeftDrive.setPower(-desiredPower);
        backRightDrive.setPower(-desiredPower);

        while (odometerRight.getCurrentPosition() > ticks) {
            try {
                sleep(25);
            } catch (Exception e) {
                throw new RuntimeException(e);
            }
        }
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }

    private void strafe(double desiredPower, int ticks) {
        imu.resetYaw();
        /*
        imu.getHeading
         if Heading >0 {
        wheel.setPower(correctionPower)
        wheel.setPower(correctionPower)
        wheel.setPower(0)
        }

         if Heading <0 {
        wheel.setPower(correctionPower)
        wheel.setPower(correctionPower)

        }
        */

        frontLeftDrive.setPower(desiredPower);
        frontRightDrive.setPower(-desiredPower);
        backLeftDrive.setPower(-desiredPower);
        backRightDrive.setPower(desiredPower);
        while (odometerAux.getCurrentPosition() < ticks) {
            try {
                sleep(25);
            } catch (Exception e) {
                throw new RuntimeException(e);
            }
        }
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }

    private void launch() {
        kicker.setPosition(0.2);
        sleep(250);
        launcherLeft.setVelocity(2500);
        sleep(2500);
        intakeWheels.setVelocity(1500);
        sleep(500);
        intakeWheels.setVelocity(0);
        sleep(900);
        intakeWheels.setVelocity(1500);
        sleep(500);
        intakeWheels.setVelocity(0);
        sleep(900);
        intakeWheels.setVelocity(1500);
        sleep(500);
        intakeWheels.setVelocity(0);
        sleep(50);
    }

    private void resetEncoders() {
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void Heading(double yaw, int angleDegrees) {
        if (yaw > 0) {

        }
        else if (yaw < 0) {
                                      
        }
    }
}