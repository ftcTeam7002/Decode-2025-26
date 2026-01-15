package org.firstinspires.ftc.teamcode;

import android.provider.ContactsContract;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "Odo test", group = "Robot")

public class odometry_test extends LinearOpMode {

    /* Declare OpMode members. */
    private IMU imu;
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotorEx launcherLeft = null;
    // Encoder ||||||||||||||||||||||||||||||||||||||||||||||||||||||||
    public DcMotor odometerLeft;
    public DcMotor odometerAux;
    // Kicker||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
    private Servo kicker = null;
    private ElapsedTime runtime = new ElapsedTime();

    static final double FORWARD_SPEED = 0.6;
    static final double REVERSE_SPEED = -0.6;

    // Ticks per inch (change for your wheel + encoder)
    public static final double TICKS_PER_INCH = 1058.34;
    // 9.425 inches per revolution


    @Override
    public void runOpMode() {

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logo = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
        RevHubOrientationOnRobot.UsbFacingDirection usb = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        // Initialize the drive system variables.
        frontLeftDrive = hardwareMap.get(DcMotor.class, "FL");
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive = hardwareMap.get(DcMotor.class, "BR");
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontRightDrive = hardwareMap.get(DcMotor.class, "FR");
        odometerAux = frontRightDrive;
        frontRightDrive.getCurrentPosition();
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backLeftDrive = hardwareMap.get(DcMotor.class, "BL");
        odometerLeft = backLeftDrive;
        backLeftDrive.getCurrentPosition();
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        launcherLeft = hardwareMap.get(DcMotorEx.class, "LL");

        kicker = hardwareMap.get(Servo.class, "kicker");


        // Drive code !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        imu.resetYaw();


        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        frontRightDrive.setPower(0);
        frontLeftDrive.setPower(0);
//        Reset odometry encoder only
//         ---------------------
        odometerLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odometerAux.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        odometerLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odometerAux.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        telemetry.addLine("Initialized — waiting for start");
        telemetry.update();
        waitForStart();

        // Drive forward until odometry wheel reaches the target
        // ---------------------

        int targetTicks = (int) (2 * TICKS_PER_INCH);
        int targetTicks2 = (int) (2 * TICKS_PER_INCH);
        // Start driving forward (simple open-loop power) main movement until while loop
        frontLeftDrive.setPower(FORWARD_SPEED);
        frontRightDrive.setPower(REVERSE_SPEED);
        backLeftDrive.setPower(FORWARD_SPEED);
        backRightDrive.setPower(REVERSE_SPEED);
        sleep(50);
        frontLeftDrive.setPower(FORWARD_SPEED);
        frontRightDrive.setPower(REVERSE_SPEED);
        backLeftDrive.setPower(REVERSE_SPEED);
        backRightDrive.setPower(FORWARD_SPEED);

        double auxposition = odometerAux.getCurrentPosition();
        double leftposition = odometerLeft.getCurrentPosition();

        while (opModeIsActive() && Math.abs(auxposition) < targetTicks && Math.abs(leftposition) < targetTicks2)  {

            YawPitchRollAngles robotorientation;
            robotorientation = imu.getRobotYawPitchRollAngles();
            double yaw = robotorientation.getYaw(AngleUnit.DEGREES);

            leftposition = odometerLeft.getCurrentPosition();
            auxposition = odometerAux.getCurrentPosition();
            telemetry.addData("yawposition", yaw);
            telemetry.addData("OdometerAux Ticks", odometerAux.getCurrentPosition());
            telemetry.addData("Odometry Ticks", odometerLeft.getCurrentPosition());
            telemetry.addData("Target Ticks", targetTicks);
            telemetry.update();
            sleep(50);
        }

        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backLeftDrive.setPower(0);

        telemetry.addLine("COMPLETE");
        telemetry.update();
        sleep(500);
        // Stop

    }
}