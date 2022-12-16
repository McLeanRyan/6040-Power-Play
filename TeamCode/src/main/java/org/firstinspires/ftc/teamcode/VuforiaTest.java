package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

@TeleOp
public class VuforiaTest extends LinearOpMode {
    VuforiaLocalizer vuforia;
    WebcamName cam;


    @Override
    public void runOpMode() throws InterruptedException {

        cam = hardwareMap.get(WebcamName.class, "cam");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AcYuy1L/////AAABmTDr3bZMPEIUlbMEDBL7zLRCTVyWmA9/6uvaVCTpjSS6NRdpMhV4PiYfzqKk0cYEknRDr5ajLiNrdtujiEowIfYMOMyHChF9e1bwAs9YMNaVzVJQpEIhQtm+kWQ/sH/hYGvJFx1B66Maa2dSV2Ddu0sllYErT1MCDlWaas0gAwCy7KnYtnHs4RGjWeOQaBI44vLkL3Wwu/ChjCUjz2vSvsDhf7CkDs/eG+5OsJ9heetbgbYb8phJpgH29Lq4FXe1oY5FYMo7/FocBy1vXnNGJkNX3YLc7AzDCaD3dUNwPoGFw3dxjjM+wzMlrw2DjCVNT/o6sbLJDz+FWa0Vk6lgeltaAxv0rTUy4FpLk9PBDV1q";
        parameters.cameraName = cam;
        this.vuforia = ClassFactory.getInstance().createVuforia(parameters);




        while (opModeIsActive()) {

        }
    }
}
