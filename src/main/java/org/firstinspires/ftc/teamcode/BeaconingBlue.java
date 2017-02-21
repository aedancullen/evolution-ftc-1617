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
 * Created by aedan on 2/20/17.
 */

@Autonomous(name="BeaconingBlue")
public class BeaconingBlue extends LinearOpMode {

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
        lineBopperRight = hardwareMap.lightSensor.get("liLineR");

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
        omni90 = hardwareMap.dcMotor.get("dcOmni90");


        colorFront = hardwareMap.colorSensor.get("colorFront");
        colorFront.enableLed(false);

        launchFlap.setPosition(0);

        waitForStart();

        EssentialHeading target;
        double gain = 0.009;
        int start;

        target = new EssentialHeading(180);
        start = omni90.getCurrentPosition();
        while (!(omni90.getCurrentPosition() < start - 2660) && opModeIsActive()) {
            fruity.driveWithRamper(target, -0.5, fruity.getNecessaryRotationPower(new EssentialHeading(0), gain));
        }
        fruity.drive(new EssentialHeading(0), 0, 0);

        launchL.setPower(LAUNCH_MOTOR_SPEED);
        launchR.setPower(LAUNCH_MOTOR_SPEED);
        sleep(1000);

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

    public boolean wallHasBopped(UltrasonicSensor boppers[], double level) {
        double greatest = -1;
        for (UltrasonicSensor bopper : boppers) {
            if (bopper.getUltrasonicLevel() > greatest) {
                greatest = bopper.getUltrasonicLevel();
            }
        }
        if (Math.abs(level - greatest) < 0.5) {
            return true;
        }
        else {
            return false;
        }
    }

    public double bopWall(UltrasonicSensor boppers[], double level, double gain) {
        double greatest = -1;
        for (UltrasonicSensor bopper : boppers) {
            if (bopper.getUltrasonicLevel() > greatest) {
                greatest = bopper.getUltrasonicLevel();
            }
        }
        return (greatest - level) * gain;
    }



}
