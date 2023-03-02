package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous
public class Park extends LinearOpMode {

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

    BNO055IMU imu;
    private Orientation angles;

    private ElapsedTime runtime = new ElapsedTime();

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

        //rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        //rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fourBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        topLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        waitForStart();

        drive(.5, 2000,false);
    }

    public void encoderDrive(double speed, double inches, double timeoutS, boolean strafe) {
        telemetry.addLine("Encoder Drive");

        int newLFTarget;
        int newRFTarget;
        int newLBTarget;
        int newRBTarget;
        int lFPos = leftFront.getCurrentPosition();
        int rFPos = rightFront.getCurrentPosition();
        int lBPos = leftBack.getCurrentPosition();
        int rBPos = rightBack.getCurrentPosition();
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, ZYX, AngleUnit.DEGREES);
        double startAngle = angles.firstAngle;

        //set motor directions if strafing
        if (strafe) {
            newLFTarget = lFPos + (int) (inches * ticksPerDriveInch);
            newRFTarget = rFPos - (int) (inches * ticksPerDriveInch);
            newLBTarget = lBPos - (int) (inches * ticksPerDriveInch);
            newRBTarget = rBPos + (int) (inches * ticksPerDriveInch);
        } else {
            newLFTarget = lFPos + (int) (inches * ticksPerDriveInch);
            newRFTarget = rFPos + (int) (inches * ticksPerDriveInch);
            newLBTarget = lBPos + (int) (inches * ticksPerDriveInch);
            newRBTarget = rBPos + (int) (inches * ticksPerDriveInch);
        }

        telemetry.addData("speed", speed);
        telemetry.addData("inches", inches);
        telemetry.addData("newLFTarget", newLFTarget);
        telemetry.addData("newRFTarget", newRFTarget);
        telemetry.addData("newLBTarget", newLBTarget);
        telemetry.addData("newRBTarget", newRBTarget);

        //Start running motors to the target position at desired speed
        leftFront.setTargetPosition(newLFTarget);
        rightFront.setTargetPosition(newRFTarget);
        leftBack.setTargetPosition(newLBTarget);
        rightBack.setTargetPosition(newRBTarget);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(Math.abs(speed));
        rightFront.setPower(Math.abs(speed));
        leftBack.setPower(Math.abs(speed));
        rightBack.setPower(Math.abs(speed));

        runtime.reset();

        while (runtime.seconds() < timeoutS && (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy())) {
            //Adjust for weight distribution offsetting the strafe by using proportional gyro correction

            telemetry.addData("LF Current Position", leftFront.getCurrentPosition());
            telemetry.addData("RF Current Position", rightFront.getCurrentPosition());
            telemetry.addData("LB Current Position", leftBack.getCurrentPosition());
            telemetry.addData("RB Current Position", rightBack.getCurrentPosition());
            telemetry.addData("LF Current Power", leftFront.getPower());
            telemetry.addData("RF Current Power", rightFront.getPower());
            telemetry.addData("LB Current Power", leftBack.getPower());
            telemetry.addData("RB Current Power", rightBack.getPower());
            telemetry.update();
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(100);

    }
    public void drive (double speed, int seconds, boolean strafe) {
        if (strafe) {
            leftFront.setPower(-speed);
            leftBack.setPower(speed);
            rightFront.setPower(speed);
            rightBack.setPower(-speed);
        }else {
            leftFront.setPower(speed);
            leftBack.setPower(speed);
            rightFront.setPower(speed);
            rightBack.setPower(speed);
        }
        sleep(seconds);
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
}
