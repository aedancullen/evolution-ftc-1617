package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;

import io.github.aedancullen.fruity.EssentialHeading;
import io.github.aedancullen.fruity.FruityController;
import io.github.aedancullen.fruity.MotorConfigurations;

@Autonomous(name="Simple Auto-launching Autonomous", group="MarvMk7")
public class LaunchAuto extends LinearOpMode {

    DcMotor omni90;
    DcMotor omni0;

    final int LAUNCH_MS_TO_WAITING = 1000; // time until waiting state can be reached (e.g. flipper fall time)
    int LAUNCH_MS_TO_WAITING_LEFT = 0;
    final int LAUNCH_STATE_WAITING = 0; // wheels off, flipper down

    final int LAUNCH_STATE_STARTING = 1; // wheels accelerating, flipper down

    final int LAUNCH_MS_TO_FLIPPING = 800; // time left until flipping state can be reached (e.g wheel accelerate time)
    int LAUNCH_MS_TO_FLIPPING_LEFT = 0;
    final int LAUNCH_STATE_FLIPPING = 2; // wheels on, flipper rising

    final int LAUNCH_MS_TO_RETRACTING = 400; // time left until retracting state can be reached (e.g. flipper rise time)
    int LAUNCH_MS_TO_RETRACTING_LEFT = 0;
    final int LAUNCH_STATE_RETRACTING = 3; // wheels on, flipper retracting

    final double FLAP_UP_POSITION = 0.5;
    final double LAUNCH_MOTOR_SPEED = 1;

    FruityController fruity;

    DcMotor collector;
    DcMotor launchL;
    DcMotor launchR;
    Servo launchFlap;

    LightSensor lightFrontL;
    LightSensor lightFrontR;

    /**public void oldinit() {
        fruity = new FruityController(hardwareMap, telemetry, "",
                Arrays.asList(
                        hardwareMap.dcMotor.get("dcOmni0"),
                        hardwareMap.dcMotor.get("dcOmni90"),
                        hardwareMap.dcMotor.get("dcOmni180"),
                        hardwareMap.dcMotor.get("dcOmni270")
                ),
                DcMotorSimple.Direction.REVERSE,
                DcMotor.RunMode.RUN_USING_ENCODER,
                MotorConfigurations.QUAD_NONDIAGONAL_SHORT);
        //fruity.setupRamper(0.002, 0.002);

        collector = hardwareMap.dcMotor.get("dcCollector0");
        launchL = hardwareMap.dcMotor.get("dcLaunchL");
        launchL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launchL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launchR = hardwareMap.dcMotor.get("dcLaunchR");
        launchR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launchR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launchR.setDirection(DcMotorSimple.Direction.REVERSE);
        launchFlap = hardwareMap.servo.get("svFlap0");
        launchFlap.setDirection(Servo.Direction.REVERSE);
        omni90 = hardwareMap.dcMotor.get("dcOmni90");
        omni0 = hardwareMap.dcMotor.get("dcOmni0");

        lightFrontL = hardwareMap.lightSensor.get("lightFrontL");
        lightFrontR = hardwareMap.lightSensor.get("lightFrontR");
    }**/

    public void runOpMode() {
        fruity = new FruityController(hardwareMap, telemetry, "",
                Arrays.asList(
                        hardwareMap.dcMotor.get("dcOmni0"),
                        hardwareMap.dcMotor.get("dcOmni90"),
                        hardwareMap.dcMotor.get("dcOmni180"),
                        hardwareMap.dcMotor.get("dcOmni270")
                ),
                DcMotorSimple.Direction.REVERSE,
                DcMotor.RunMode.RUN_USING_ENCODER,
                DcMotor.ZeroPowerBehavior.BRAKE,
                MotorConfigurations.QUAD_NONDIAGONAL_SHORT, 0.002, 0.01, 0.1);
        //fruity.setupRamper(0.002, 0.002);

        collector = hardwareMap.dcMotor.get("dcCollector0");
        launchL = hardwareMap.dcMotor.get("dcLaunchL");
        launchL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launchL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launchR = hardwareMap.dcMotor.get("dcLaunchR");
        launchR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launchR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launchR.setDirection(DcMotorSimple.Direction.REVERSE);
        launchFlap = hardwareMap.servo.get("svFlap0");
        launchFlap.setDirection(Servo.Direction.REVERSE);
        omni90 = hardwareMap.dcMotor.get("dcOmni90");


        waitForStart();



        int starte = omni90.getCurrentPosition();
        fruity.drive(new EssentialHeading(0), -0.2, 0);
        while (!(omni90.getCurrentPosition() > starte + 2280)) {try {Thread.sleep(1);if (isStopRequested()) {finish();}}catch (InterruptedException e) {} }
        fruity.drive(new EssentialHeading(0), 0, 0);

        long start = System.currentTimeMillis();
        launchL.setPower(LAUNCH_MOTOR_SPEED);
        launchR.setPower(LAUNCH_MOTOR_SPEED);
        while (!(System.currentTimeMillis() > start + 800)) {try {Thread.sleep(1);if (isStopRequested()) {finish();}}catch (InterruptedException e) {} }

        start = System.currentTimeMillis();
        launchFlap.setPosition(FLAP_UP_POSITION);
        while (!(System.currentTimeMillis() > start + 400)) {try {Thread.sleep(1);if (isStopRequested()) {finish();}}catch (InterruptedException e) {} }
        start = System.currentTimeMillis();
        launchFlap.setPosition(0);
        while (!(System.currentTimeMillis() > start + 1000)) {try {Thread.sleep(1);if (isStopRequested()) {finish();}}catch (InterruptedException e) {} }

        start = System.currentTimeMillis();
        launchFlap.setPosition(FLAP_UP_POSITION);
        while (!(System.currentTimeMillis() > start + 400)) {try {Thread.sleep(1);if (isStopRequested()) {finish();}}catch (InterruptedException e) {} }


        launchFlap.setPosition(0);
        launchL.setPower(0);
        launchR.setPower(0);

        /**
        starte = omni90.getCurrentPosition();
        fruity.drive(new EssentialHeading(0), -0.2, 0);
        while (!(omni90.getCurrentPosition() > starte + 3000)) {try {Thread.sleep(1);if (isStopRequested()) {finish();}}catch (InterruptedException e) {} }
        fruity.drive(new EssentialHeading(0), 0, 0);
         **/

        starte = omni0.getCurrentPosition();


    }

    public void finish() {
        stop();
    }

}
