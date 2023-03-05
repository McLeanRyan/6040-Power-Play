package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Autonomous
public class RightSideConeDropPark extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

    private DcMotor fourBar;
    private DcMotor topLift;
    private DcMotor bottomLift;

    private CRServo claw;

    public static final double ticksPerMotorRev = 537.7;
    public static final double driveGearReduction = 1;
    public static final double wheelDiameterInches = 3.78;
    public static final double ticksPerDriveInch = (ticksPerMotorRev * driveGearReduction) / (wheelDiameterInches * 3.14159265359);
    public static final double groundJunctionHeight = 0;
    public static final double lowJunctionHeight = 0;
    public static final double midJunctionHeight = 0;
    public static final double highJunctionHeight = 0;

    private WebcamName webcam;

    private VuforiaLocalizer vuforia = null;
    private VuforiaTrackables targets = null;

    public static final String JustinRobodogs23 = "AbPAsZf/////AAABmfibwXNw70SrjNg5qBWQi6ohhZf4HQXyJCn4vbC0aLRr+NHm3MyUJXkJxUF2Wk4RqQrqcoJCA3fELgH4SrjbvnsQzMFFb0y/GtXrHfYwzwbVG9Gg3LrOd/Rlet/qI39Q9foADM6Zu9XV21KISqXKamo6DDV8BfOE8vz6z18j7O4hoUfX9JYidlFunwAUFNMvHw5KEreXxAdKO6V2s51kUN1Jus7D9SKsztg7gIlU6D2BC2o7SXu0x8sN2/EqYcNGt9UpeV8SCYXImiIHN3eMzF9U4VKUUzYOzjuU2L+04BByIEtbCZKO2wPwCsK7WSnq65ES1KnO669ZOwt8dudWiE1Pl3dMOkisVmXx23UMrt5J"; //License Key


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

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fourBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        topLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters webcamparameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId); //R.id
        webcamparameters.vuforiaLicenseKey = JustinRobodogs23;
        webcamparameters.cameraName = webcam;
        webcamparameters.useExtendedTracking = false;

        vuforia = ClassFactory.getInstance().createVuforia(webcamparameters);

        targets = this.vuforia.loadTrackablesFromAsset("JustinFTC23");
        targets.get(0).setName("Sleepy");
        targets.get(2).setName("Catan");
        targets.get(1).setName("Triangle");

        targets.activate();

        waitForStart();

        VuforiaTrackableDefaultListener Sleepy = (VuforiaTrackableDefaultListener) targets.get(0).getListener();
        VuforiaTrackableDefaultListener Catan = (VuforiaTrackableDefaultListener) targets.get(2).getListener();
        VuforiaTrackableDefaultListener Triangle = (VuforiaTrackableDefaultListener) targets.get(1).getListener();

        telemetry.addLine("Vuforia Initialized");
        telemetry.update();

        while(opModeIsActive()) {

/*
            encoderDrive(.4, -24, 30, true, 0.5, 0.7); //right strafe positive
            stop();
*/

            claw.setPower(.5);
            sleep(1000);

            fourBar.setPower(.7);
            sleep(500);


            if (Triangle.isVisible()) {
                telemetry.addLine("Triforce");
                telemetry.update();
                park(0);
                stop();
            } else if (Catan.isVisible()) {
                telemetry.addLine("Catan");
                telemetry.update();
                park(1);
                stop();
            } else if (Sleepy.isVisible()) {
                telemetry.addLine("Sleepy");
                telemetry.update();
                park(2);
                stop();
            }
        }
    }

    public void encoderDrive(double speed, double inches, double timeoutS, boolean strafe, double clawPower, double fourBarPower) {
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

        claw.setPower(clawPower);
        fourBar.setPower(fourBarPower);

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

    public void park (int sleeveCase) {
        claw.setPower(.5);
        fourBar.setPower(.7);

        encoderDrive(.4, 17, 30, true, .5, .7);
        sleep(250);
        //fourBar.setPower(.4);
        encoderDrive(.4, 16.5, 30, false, .5, .7);
        sleep(250);
        fourBar.setPower(-0.2);
        sleep(500);
        claw.setPower(-.5);
        sleep(500);
        claw.setPower(0);
        encoderDrive(.4, -19, 30, false, 0, 0);
        encoderDrive(.4, 1, 30, false, 0, 0);

        if (sleeveCase == 2) {
            encoderDrive(.4, 13, 30, true, 0, 0);
            encoderDrive(.4, 26, 30, false, 0, 0);
        }else if (sleeveCase == 1) {
            encoderDrive(.4, -14.5, 30, true, 0, 0);
            encoderDrive(.4, 26, 30, false, 0, 0);
        }else if (sleeveCase == 0) {
            encoderDrive(.4, -39.5, 30, true, 0, 0);
            encoderDrive(.4, 26, 30, false, 0, 0);
        }else {
            encoderDrive(.4, 13, 30, true, 0, 0);
        }
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
