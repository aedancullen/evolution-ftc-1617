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

        EssentialHeading target;
        double gain = 0.009;
        int start;

        // Out to launch pos
        target = new EssentialHeading(180);
        start = omni90.getCurrentPosition();
        while (!(omni90.getCurrentPosition() < start - 2660) && opModeIsActive()) {
            fruity.driveWithRamper(target, -0.5, fruity.getNecessaryRotationPower(new EssentialHeading(0), gain));
        }
        fruity.drive(new EssentialHeading(0), 0, 0);

        // Launch routine
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

        // Out to center of two beacons
        target = new EssentialHeading(-40);
        start = omni90.getCurrentPosition();
        while (!(omni90.getCurrentPosition() < start - 6100) && opModeIsActive()) {
            fruity.driveWithRamper(target, -0.5, fruity.getNecessaryRotationPower(new EssentialHeading(0), gain));
        }
        fruity.drive(new EssentialHeading(0), 0, 0);

        // Rotate to face beacons with back
        while (!fruity.isFacing(target) && opModeIsActive()) {
            fruity.driveWithRamper(target, 0, fruity.getNecessaryRotationPower(new EssentialHeading(90), gain));
        }
        fruity.drive(new EssentialHeading(0), 0, 0);

        // Get correct distance
        target = new EssentialHeading(-90);
        start = omni90.getCurrentPosition();
        while ( !(maxMeasure(new UltrasonicSensor[] {wallBopperRight, wallBopperLeft}) <= 30) && opModeIsActive()) {
            fruity.driveWithRamper(target, 0.1, fruity.getNecessaryRotationPower(new EssentialHeading(90), gain));
        }

        // Move to the line of the first beacon
        target = new EssentialHeading(0);
        start = omni90.getCurrentPosition();
        while ( !(lineBopperLeft.getLightDetected() > lineBopperRight.getLightDetected() + 10) && opModeIsActive()) {
            fruity.driveWithRamper(target, 0.2, fruity.getNecessaryRotationPower(new EssentialHeading(90), gain));
        }
        while ( !(Math.abs(lineBopperRight.getLightDetected() - lineBopperLeft.getLightDetected()) < 10) && opModeIsActive()) {
            fruity.driveWithRamper(target, 0.2, fruity.getNecessaryRotationPower(new EssentialHeading(90), gain));
        }
        fruity.drive(new EssentialHeading(0), 0, 0);

        // Get correct distance
        target = new EssentialHeading(-90);
        start = omni90.getCurrentPosition();
        while ( !(maxMeasure(new UltrasonicSensor[] {wallBopperRight, wallBopperLeft}) <= 8) && opModeIsActive()) {
            fruity.driveWithRamper(target, 0.1, fruity.getNecessaryRotationPower(new EssentialHeading(90), gain));
        }

        // Push beacon
        doPush();

        // Get correct distance
        target = new EssentialHeading(-90);
        start = omni90.getCurrentPosition();
        while ( !(maxMeasure(new UltrasonicSensor[] {wallBopperRight, wallBopperLeft}) >= 30) && opModeIsActive()) {
            fruity.driveWithRamper(target, -0.1, fruity.getNecessaryRotationPower(new EssentialHeading(90), gain));
        }

        // Assure that we're away from the line
        target = new EssentialHeading(0);
        start = omni0.getCurrentPosition();
        while (!(omni0.getCurrentPosition() < start - 500) && opModeIsActive()) {
            fruity.driveWithRamper(target, -0.2, fruity.getNecessaryRotationPower(new EssentialHeading(0), gain));
        }

        // Move to the line of the second beacon
        target = new EssentialHeading(0);
        start = omni90.getCurrentPosition();
        while ( !(lineBopperRight.getLightDetected() > lineBopperLeft.getLightDetected() + 10) && opModeIsActive()) {
            fruity.driveWithRamper(target, -0.2, fruity.getNecessaryRotationPower(new EssentialHeading(90), gain));
        }
        while ( !(Math.abs(lineBopperLeft.getLightDetected() - lineBopperRight.getLightDetected()) < 10) && opModeIsActive()) {
            fruity.driveWithRamper(target, -0.2, fruity.getNecessaryRotationPower(new EssentialHeading(90), gain));
        }
        fruity.drive(new EssentialHeading(0), 0, 0);

        // Get correct distance
        target = new EssentialHeading(-90);
        start = omni90.getCurrentPosition();
        while ( !(maxMeasure(new UltrasonicSensor[] {wallBopperRight, wallBopperLeft}) <= 8) && opModeIsActive()) {
            fruity.driveWithRamper(target, 0.1, fruity.getNecessaryRotationPower(new EssentialHeading(90), gain));
        }

        //Push beacon
        doPush();

    }


    public double maxMeasure(UltrasonicSensor boppers[]) {
        double greatest = -1;
        for (UltrasonicSensor bopper : boppers) {
            if (bopper.getUltrasonicLevel() > greatest) {
                greatest = bopper.getUltrasonicLevel();
            }
        }
        return greatest;
    }

    public void doPush() {

        EssentialHeading target;
        double gain = 0.009;
        int start;


        colorFront.enableLed(false);
        if (colorFront.blue() > colorFront.red()) {
            // good, slide left a tad
            target = new EssentialHeading(0);
            start = omni0.getCurrentPosition();
            while (!(omni0.getCurrentPosition() < start - 300) && opModeIsActive()) {
                fruity.driveWithRamper(target, -0.1, fruity.getNecessaryRotationPower(new EssentialHeading(-90), gain));
            }
            fruity.drive(new EssentialHeading(0), 0, 0);
        } else {
            // not good, slide right a tad
            target = new EssentialHeading(0);
            start = omni0.getCurrentPosition();
            while (!(omni0.getCurrentPosition() > start + 300) && opModeIsActive()) {
                fruity.driveWithRamper(target, 0.1, fruity.getNecessaryRotationPower(new EssentialHeading(-90), gain));
            }
            fruity.drive(new EssentialHeading(0), 0, 0);
        }

        // bump it
        // forward
        target = new EssentialHeading(90);
        start = omni90.getCurrentPosition();
        while (!(omni90.getCurrentPosition() < start - 400) && opModeIsActive()) {
            fruity.driveWithRamper(target, 0.1, fruity.getNecessaryRotationPower(new EssentialHeading(-90), gain));
        }
        fruity.drive(new EssentialHeading(0), 0, 0);

        // reverse
        target = new EssentialHeading(-90);
        start = omni90.getCurrentPosition();
        while (!(omni90.getCurrentPosition() > start + 400) && opModeIsActive()) {
            fruity.driveWithRamper(target, 0.1, fruity.getNecessaryRotationPower(new EssentialHeading(-90), gain));
        }
        fruity.drive(new EssentialHeading(0), 0, 0);
    }

}
