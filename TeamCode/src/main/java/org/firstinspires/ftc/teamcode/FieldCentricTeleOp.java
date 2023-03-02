package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class FieldCentricTeleOp extends LinearOpMode {
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

    private DcMotor fourBar;
    private DcMotor topLift;
    private DcMotor bottomLift;

    private CRServo claw;

    public static final double ticksPerMotorRev = 383.6;
    public static final double driveGearReduction = 0.5;
    public static final double wheelDiameterInches = 4;
    public static final double ticksPerDriveInch = (ticksPerMotorRev * driveGearReduction) / (wheelDiameterInches * 3.14159265359);
    public static final double groundJunctionHeight = 0;
    public static final double lowJunctionHeight = 0;
    public static final double midJunctionHeight = 0;
    public static final double highJunctionHeight = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");

        fourBar = hardwareMap.dcMotor.get("fourBar");
        topLift = hardwareMap.dcMotor.get("topLift");
        bottomLift = hardwareMap.dcMotor.get("bottomLift");

        claw = hardwareMap.crservo.get("claw");

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fourBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        topLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            double y = gamepad1.left_stick_y; // Remember, this is reversed!
            double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            if (Math.abs(y) < 0.05) y = 0;
            else if (Math.abs(y) < 0.25) y /= 2;

            if (Math.abs(x) < 0.05) x = 0;
            else if (Math.abs(x) < 0.25) x /= 2;

            if (Math.abs(rx) < 0.05) rx = 0;
            else if(Math.abs(rx) < 0.25) rx /= 2;

            // Read inverse IMU heading, as the IMU heading is CW positive
            double botHeading = -imu.getAngularOrientation().firstAngle;

            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            leftFront.setPower(.75*frontLeftPower);
            leftBack.setPower(.75*backLeftPower);
            rightFront.setPower(.75*frontRightPower);
            rightBack.setPower(.75*backRightPower);

            topLift.setPower(-gamepad2.left_stick_y);
            bottomLift.setPower(-gamepad2.left_stick_y);
            fourBar.setPower(-gamepad2.right_stick_y);

            while (gamepad2.right_bumper) {
                claw.setPower(.5);
            }
            while (gamepad2.left_bumper) {
                claw.setPower(-.5);
            }
            if (gamepad2.a) {
                topLift.setPower(1);
                bottomLift.setPower(1);
            }
            if (gamepad2.b) {
                topLift.setPower(-1);
                bottomLift.setPower(-1);
            }
        }
    }

    public void runLift (int junctionHeight) {
        int newTopLiftTarget = 0;
        int newBottomLiftTarget = 0;
        int topLiftPos = topLift.getCurrentPosition();
        int bottomLiftPos = bottomLift.getCurrentPosition();

        topLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (junctionHeight == 0) {
            newTopLiftTarget = (int) (groundJunctionHeight);
            newBottomLiftTarget = (int) (groundJunctionHeight);
        }else if (junctionHeight == 1) {
            newTopLiftTarget = (int) (lowJunctionHeight);
            newBottomLiftTarget = (int) (lowJunctionHeight);
        }else if (junctionHeight == 2) {
            newTopLiftTarget = (int) (midJunctionHeight);
            newBottomLiftTarget = (int) (midJunctionHeight);
        }else if (junctionHeight == 3){
            newTopLiftTarget = (int) (highJunctionHeight);
            newBottomLiftTarget = (int) (highJunctionHeight);
        }

        telemetry.addData("Height Case", junctionHeight);
        telemetry.addData("newTopLiftTarget",  newTopLiftTarget);
        telemetry.addData("newBottomLiftTarget", newBottomLiftTarget);

        topLift.setTargetPosition(newTopLiftTarget);
        bottomLift.setTargetPosition(newBottomLiftTarget);

        topLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        topLift.setPower(1);
        bottomLift.setPower(1);

    }
}
