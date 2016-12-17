package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by aedan on 12/17/16.
 */

@TeleOp(name="LaunchEncoderCalib")
public class LaunchEncoderMeasure extends LinearOpMode {

    DcMotor launchL;
    DcMotor launchR;

    public void runOpMode() {
        launchL = hardwareMap.dcMotor.get("dcLaunchL");
        launchL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //launchL.setMaxSpeed(launchL.getMaxSpeed() / 2);
        launchL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launchR = hardwareMap.dcMotor.get("dcLaunchR");
        launchR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //launchR.setMaxSpeed(launchR.getMaxSpeed() / 2);
        launchR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launchR.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        launchL.setPower(1);
        launchR.setPower(1);
        sleep(1000);

        int lTicks = launchL.getCurrentPosition();
        int rTicks = launchR.getCurrentPosition();

        launchL.setPower(0);
        launchR.setPower(0);

        telemetry.addData("lTicks", lTicks);
        telemetry.addData("rTicks", rTicks);
        telemetry.update();

        sleep(5000);

    }

}
