package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

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
    public static final double turningInches = 355;
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

//        strafe(0.3, (int) (72 * strafeInches));
//        sleep(50);
//        resetEncoders();
//        launch();
          turnLeft(0.3, 90, (int) (turningInches));
          sleep(50);
          resetEncoders();


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
       stopMotors();

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
        stopMotors();
    }

    private void strafe(double desiredPower, int ticks) {
        imu.resetYaw();
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double targetYaw = orientation.getYaw(AngleUnit.DEGREES);

        while (odometerAux.getCurrentPosition() < ticks && opModeIsActive()) {
            orientation = imu.getRobotYawPitchRollAngles();
            headingCorrection(orientation.getYaw(AngleUnit.DEGREES), targetYaw);
            telemetry.addData("targetYaw", targetYaw);
            telemetry.addData("currentYaw", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.update();
            try {
                sleep(25);
            } catch (Exception e) {
                throw new RuntimeException(e);
            }
        }
        stopMotors();
    }
    private void turnLeft(double desiredPower, double desiredYaw, int turningTicks) {
        imu.resetYaw();
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double targetYaw = orientation.getYaw(AngleUnit.DEGREES);
        boolean done = false;
        while(!done && opModeIsActive()){
            double currentYaw= imu.getRobotYawPitchRollAngles().getYaw();
            if (Math.abs(currentYaw - desiredYaw)<1){
                done = true;
                break;
            }
            if (currentYaw < desiredYaw) {
                frontLeftDrive.setPower(desiredPower);
                frontRightDrive.setPower(-desiredPower);
                backLeftDrive.setPower(desiredPower);
                backRightDrive.setPower(-desiredPower);
            }
            if (currentYaw > desiredYaw){
                frontLeftDrive.setPower(-desiredPower);
                frontRightDrive.setPower(desiredPower);
                backLeftDrive.setPower(-desiredPower);
                backRightDrive.setPower(desiredPower);
            }
            telemetry.addData("Yaw", orientation.getYaw());
        }

       /* while (odometerAux.getCurrentPosition() < turningTicks && opModeIsActive()) {
            if (currentYaw != desiredYaw) {
                while(opModeIsActive()) {
                    telemetry.addLine("Turning Left");
                    telemetry.addData("currentYaw", orientation.getYaw(AngleUnit.DEGREES));
                    telemetry.addData("OdoAux", odometerAux.getCurrentPosition());
                    telemetry.update();

                    frontLeftDrive.setPower(desiredPower);
                    frontRightDrive.setPower(-desiredPower);
                    backLeftDrive.setPower(desiredPower);
                    backRightDrive.setPower(-desiredPower);
                }


            }
            else{
                stopMotors();
            }
            if ( currentYaw > desiredYaw); {
                telemetry.addLine("Turning Right");

            }
        }
        stopMotors();  */
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

    private void headingCorrection(double currentYaw, double desiredYaw) {
     if (Math.abs(currentYaw - desiredYaw)<1){
         telemetry.addLine("deadzone: <1");
         frontLeftDrive.setPower(0.3);
         frontRightDrive.setPower(-0.3);
         backLeftDrive.setPower(-0.3);
         backRightDrive.setPower(0.3);
        sleep(50);
     }
       else if (currentYaw < desiredYaw) {
            telemetry.addLine("rotating: left");
            frontLeftDrive.setPower(0.3); // in
            backLeftDrive.setPower(-0.4); //out
            frontRightDrive.setPower(-0.3); // in
            backRightDrive.setPower(0.4); // out
        }
        else if (currentYaw > desiredYaw) {
            telemetry.addLine("rotating: right");
            frontRightDrive.setPower(-0.4);
            backRightDrive.setPower(0.3);
            backLeftDrive.setPower(-0.3);
            frontLeftDrive.setPower(0.4);

        }

        }

    private void stopMotors(){
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontLeftDrive.setPower(0);
    }
}