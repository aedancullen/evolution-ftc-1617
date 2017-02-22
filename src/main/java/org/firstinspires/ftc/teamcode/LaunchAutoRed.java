package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import java.util.Arrays;

import io.github.aedancullen.fruity.EssentialHeading;
import io.github.aedancullen.fruity.FruityController;
import io.github.aedancullen.fruity.MotorConfigurations;

/**
 * Created by aedan on 12/17/16.
 */

@Autonomous(name="OnlyLaunchRed")
public class LaunchAutoRed extends LinearOpMode {


    DcMotor omni0;
    DcMotor omni90;

    final double FLAP_UP_POSITION = 0.5;
    final double LAUNCH_MOTOR_SPEED = 0.82;

    FruityController fruity;

    DcMotor launchL;
    DcMotor launchR;
    Servo launchFlap;

    ColorSensor colorFront;

    LightSensor lineBopperLeft;
    LightSensor lineBopperRight;


    UltrasonicSensor wallBopperLeft;
    UltrasonicSensor wallBopperRight;

    public void runOpMode() {
        lineBopperLeft = hardwareMap.lightSensor.get("liLineL");
        lineBopperLeft.enableLed(true);
        lineBopperRight = hardwareMap.lightSensor.get("liLineR");
        lineBopperRight.enableLed(true);

        wallBopperLeft = hardwareMap.ultrasonicSensor.get("ulWallL");
        wallBopperRight = hardwareMap.ultrasonicSensor.get("ulWallR");

        fruity = new FruityController(hardwareMap, telemetry, "imu",
                Arrays.asList(
                        hardwareMap.dcMotor.get("dcOmni0"),
                        hardwareMap.dcMotor.get("dcOmni90"),
                        hardwareMap.dcMotor.get("dcOmni180"),
                        hardwareMap.dcMotor.get("dcOmni270")
                ),
                DcMotorSimple.Direction.REVERSE,
                DcMotor.RunMode.RUN_USING_ENCODER,
                MotorConfigurations.QUAD_NONDIAGONAL_SHORT);
        //fruity.setupRamper(0.001, 0.001, false);

        launchL = hardwareMap.dcMotor.get("dcLaunchL");
        launchL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchL.setMaxSpeed(2700);
        launchL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launchR = hardwareMap.dcMotor.get("dcLaunchR");
        launchR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchR.setMaxSpeed(2700);
        launchR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launchR.setDirection(DcMotorSimple.Direction.REVERSE);
        launchFlap = hardwareMap.servo.get("svFlap0");
        launchFlap.setDirection(Servo.Direction.REVERSE);
        omni0 = hardwareMap.dcMotor.get("dcOmni0");
        omni90 = hardwareMap.dcMotor.get("dcOmni90");


        colorFront = hardwareMap.colorSensor.get("colorFront");
        colorFront.enableLed(false);

        launchFlap.setPosition(0);

        waitForStart();

        telemetry.addData("Waiting", "for 15 seconds");
        telemetry.update();

        sleep(15000);

        EssentialHeading target;
        double gain = 0.009;
        int start;

        telemetry.addData("Waiting", "Now boppin'!");
        telemetry.update();

        launchL.setPower(LAUNCH_MOTOR_SPEED);
        launchR.setPower(LAUNCH_MOTOR_SPEED);

        // Out to launch pos
        target = new EssentialHeading(140);
        start = omni90.getCurrentPosition();
        while (!(omni90.getCurrentPosition() > start + 2800) && opModeIsActive()) {
            fruity.driveWithRamper(target, 0.6, fruity.getNecessaryRotationPower(new EssentialHeading(0), gain));
        }
        fruity.drive(new EssentialHeading(0), 0, 0);

        // Launch routine
        sleep(1000); // some extra boppers

        launchFlap.setPosition(FLAP_UP_POSITION);
        sleep(300);
        launchFlap.setPosition(0);
        sleep(1100);

        launchFlap.setPosition(FLAP_UP_POSITION);
        sleep(300);


        launchFlap.setPosition(0);
        launchL.setPower(0);
        launchR.setPower(0);



    }
}