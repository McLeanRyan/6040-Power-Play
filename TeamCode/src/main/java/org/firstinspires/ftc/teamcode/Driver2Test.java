package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class Driver2Test extends OpMode {
    private DcMotor fourBar;
    private DcMotor rightBack;
    @Override
    public void init() {
        fourBar = hardwareMap.get(DcMotor.class,"fourBar");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        fourBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        fourBar.setPower(gamepad2.left_stick_y);
        rightBack.setPower(gamepad2.right_stick_y);
        telemetry.addData("4Bar Power:", fourBar.getPower());
        telemetry.addData("rightBack Power:", rightBack.getPower());
        telemetry.update();
    }
}
