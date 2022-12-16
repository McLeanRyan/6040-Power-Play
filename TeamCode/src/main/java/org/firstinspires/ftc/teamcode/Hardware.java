package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Hardware {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // declare hardware

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

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public Hardware (LinearOpMode opmode) {
        myOpMode = opmode;
    }


    public void init()    {
        //Hardware Map
        leftFront  = myOpMode.hardwareMap.get(DcMotor.class, "leftFront");
        leftBack  = myOpMode.hardwareMap.get(DcMotor.class, "leftBack");
        rightFront  = myOpMode.hardwareMap.get(DcMotor.class, "rightFront");
        rightBack  = myOpMode.hardwareMap.get(DcMotor.class, "rightBack");
        lift1  = myOpMode.hardwareMap.get(DcMotor.class, "lift1");
        lift2  = myOpMode.hardwareMap.get(DcMotor.class, "lift2");
        vBar  = myOpMode.hardwareMap.get(DcMotor.class, "vBar");

        //Reverse inverted motors
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        lift2.setDirection(DcMotorSimple.Direction.REVERSE);

        //Set motors to break on stop
        // Why are you trying to break the motors? Aren't those kind of expensive? Trying to sabotage your own team?
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Initialize motor encoders
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        vBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        myOpMode.telemetry.addLine("Hardware Initialized");
        myOpMode.telemetry.update();
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
            newLFTarget = lFPos + (int) (inches * ticksPerInch);
            newRFTarget = rFPos - (int) (inches * ticksPerInch);
            newLBTarget = lBPos - (int) (inches * ticksPerInch);
            newRBTarget = rBPos + (int) (inches * ticksPerInch);
        } else {
            newLFTarget = lFPos + (int) (inches * ticksPerInch);
            newRFTarget = rFPos + (int) (inches * ticksPerInch);
            newLBTarget = lBPos + (int) (inches * ticksPerInch);
            newRBTarget = rBPos + (int) (inches * ticksPerInch);
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
            if (!strafe) {
                double error = kP * (startAngle - angles.firstAngle);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, ZYX, AngleUnit.DEGREES);
                telemetry.addData("Start Angle", startAngle);
                telemetry.addData("Current Angle", angles.firstAngle);
                telemetry.addData("error", error);
                telemetry.update();
                if (error > 0) {
                    leftFront.setPower(Math.abs(speed) - error);
                    rightFront.setPower(Math.abs(speed) + error);
                    leftBack.setPower(Math.abs(speed) - error);
                    rightBack.setPower(Math.abs(speed) + error);
                } else if (error < 0) {
                    leftFront.setPower(Math.abs(speed) + error);
                    rightFront.setPower(Math.abs(speed) - error);
                    leftBack.setPower(Math.abs(speed) + error);
                    rightBack.setPower(Math.abs(speed) - error);
                }
            }
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

        try {
            sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

    }

    private void gyroTurn(double targetAngle) {
        //+ is counter-clockwise
        //- is clockwise
        boolean finished = false;
        while (!finished) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, ZYX, AngleUnit.DEGREES);
            double currentAngle = angles.firstAngle;
            double e = (targetAngle - currentAngle);
            totalError += e;
            double error = (kP * e) - (kD * (currentAngle - lastAngle)) + (kI) * (totalError);
            lastAngle = currentAngle;
            leftFront.setPower(-error);
            rightFront.setPower(error);
            leftBack.setPower(-error);
            rightBack.setPower(error);
            telemetry.addData("targetAngle", targetAngle);
            telemetry.addData("currentAngle", currentAngle);
            telemetry.addData("error", error);
            telemetry.addData("targetAngle - currentAngle", targetAngle - currentAngle);
            telemetry.addData("finished", finished);



            telemetry.addData("LFM Current Power", leftFront.getPower());
            telemetry.addData("RFM Current Power", rightFront.getPower());
            telemetry.addData("LBM Current Power", leftBack.getPower());
            telemetry.addData("RBM Current Power", rightBack.getPower());

            telemetry.update();
            if (Math.abs(targetAngle - currentAngle) < 4) {
                finished = true;
                telemetry.addData("Finished", finished);
                rightFront.setPower(0);
                rightBack.setPower(0);
                leftFront.setPower(0);
                leftBack.setPower(0);
                try {
                    sleep(500);
                } catch (InterruptedException ex) {
                    ex.printStackTrace();
                }
            }
        }
    }

    public void runLift(double power, int height, double timeoutS) {

    }
}
