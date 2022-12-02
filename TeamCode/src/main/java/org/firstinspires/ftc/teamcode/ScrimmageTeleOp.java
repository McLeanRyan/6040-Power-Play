package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class ScrimmageTeleOp extends LinearOpMode {
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private DcMotor lift1 = null;
    private DcMotor lift2 = null;
    private DcMotor vBar = null;

    //declare constants

    public static final double liftPower = 0.5 ;

    public static final double ticksPerMotorRev = 383.6;
    public static final double driveGearReduction = 0.5;
    public static final double wheelDiameterInches = 4;
    public static final double ticksPerInch = (ticksPerMotorRev * driveGearReduction) / (wheelDiameterInches * 3.14159265359);

    //Define imu and gyro pid constants
    BNO055IMU imu;
    private Orientation angles;

    public static final double kP = 0.005;
    public static final double kD = 0.01;
    public static final double kI = 0.00008;

    public double totalError = 0;
    public double lastAngle = 0;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        Hardware hardware = new Hardware(this);

        waitForStart();
        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rot = gamepad1.right_stick_x;

            double leftFrontPower = (y+x+rot);
            double leftBackPower = (y-x+rot);
            double rightFrontPower = (y-x-rot);
            double rightBackPower = (y+x+rot);

            leftFront.setPower(leftFrontPower);
            leftBack.setPower(leftBackPower);
            rightFront.setPower(rightFrontPower);
            rightBack.setPower(rightBackPower);

            vBar.setPower(-gamepad2.left_stick_y);

            while(gamepad2.right_bumper) {
                lift1.setPower(1);
                lift2.setPower(1);
            }

            while (gamepad2.left_bumper) {
                lift1.setPower(-.75);
                lift1.setPower(-.75);
            }
            
        }
    }
}
