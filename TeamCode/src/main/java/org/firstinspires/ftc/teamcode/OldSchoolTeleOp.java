package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class OldSchoolTeleOp extends LinearOpMode {
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
        topLift = hardwareMap.dcMotor.get("liftTop");
        bottomLift = hardwareMap.dcMotor.get("liftBottom");

        claw = hardwareMap.crservo.get("clawIntake");

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
            telemetry.addLine("Arcade Drive");
            double px = -gamepad1.left_stick_x;
            if (Math.abs(px) < 0.05) px = 0;
            double pa = gamepad1.left_stick_y;
            if (Math.abs(pa) < 0.05) pa = 0;
            double py = (gamepad1.right_stick_x*(.70));
            if (Math.abs(py) < 0.05) py = 0;
            double plf = -px + py - pa;
            double plb = px + py  - pa;
            double prf = -px + py + pa;
            double prb = px + py + pa;
            double max = Math.max(1.0, Math.abs(plf));
            max = Math.max(max, Math.abs(plb));
            max = Math.max(max, Math.abs(prf));
            max = Math.max(max, Math.abs(prb));
            plf /= max;
            plb /= max;
            prf /= max;
            prb /= max;
            leftFront.setPower(plf);
            leftBack.setPower(plb);
            rightFront.setPower(prf);
            rightBack.setPower(prb);

            topLift.setPower(-gamepad2.left_stick_y);
            bottomLift.setPower(-gamepad2.left_stick_y);
            fourBar.setPower(-gamepad2.right_stick_y);

            while (gamepad2.right_bumper) {
                claw.setPower(1);
            }
            while (gamepad2.left_bumper) {
                claw.setPower(-1);
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
