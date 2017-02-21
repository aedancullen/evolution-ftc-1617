package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@TeleOp(name="Measurement")

public class Measurement extends OpMode {

    List<DcMotor> motors;

    LightSensor lineBopperLeft;
    LightSensor lineBopperRight;


    UltrasonicSensor wallBopperLeft;
    UltrasonicSensor wallBopperRight;

    public void init() {

        lineBopperLeft = hardwareMap.lightSensor.get("liLineL");
        lineBopperLeft.enableLed(true);
        lineBopperRight = hardwareMap.lightSensor.get("liLineR");
        lineBopperRight.enableLed(true);

        wallBopperLeft = hardwareMap.ultrasonicSensor.get("ulWallL");
        wallBopperRight = hardwareMap.ultrasonicSensor.get("ulWallR");

        motors = Arrays.asList(
                hardwareMap.dcMotor.get("dcOmni0"),
                hardwareMap.dcMotor.get("dcOmni90"),
                hardwareMap.dcMotor.get("dcOmni180"),
                hardwareMap.dcMotor.get("dcOmni270"));

        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor.setPower(0);
        }


    }

    public void loop() {
        for (int i = 0; i < motors.size(); i++) {
            telemetry.addData("Motor " + i + ":", motors.get(i).getCurrentPosition());
            telemetry.addData("lineBopperLeft", lineBopperLeft.getLightDetected());
            telemetry.addData("lineBopperRight", lineBopperRight.getLightDetected());
            telemetry.addData("wallBopperRight", wallBopperRight.getUltrasonicLevel());
            telemetry.addData("wallBopperLeft", wallBopperLeft.getUltrasonicLevel());
        }
        telemetry.update();
    }

}
