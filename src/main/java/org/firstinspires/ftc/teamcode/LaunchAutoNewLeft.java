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

@Autonomous(name="Auto-launching Autonomous - LEFT/RED", group="MarvMk7")
public class LaunchAutoNewLeft extends LinearOpMode {

    DcMotor omni90;
    DcMotor omni0;

    final double FLAP_UP_POSITION = 0.5;
    final double LAUNCH_MOTOR_SPEED = 0.92;

    FruityController fruity;

    DcMotor collector;
    DcMotor launchL;
    DcMotor launchR;
    Servo launchFlap;

    ColorSensor colorFront;

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

     }**/

    public void runOpMode() {
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

        collector = hardwareMap.dcMotor.get("dcCollector0");
        launchL = hardwareMap.dcMotor.get("dcLaunchL");
        launchL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchL.setMaxSpeed(13000);
        launchL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launchR = hardwareMap.dcMotor.get("dcLaunchR");
        launchR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchR.setMaxSpeed(13000);
        launchR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launchR.setDirection(DcMotorSimple.Direction.REVERSE);
        launchFlap = hardwareMap.servo.get("svFlap0");
        launchFlap.setDirection(Servo.Direction.REVERSE);
        omni90 = hardwareMap.dcMotor.get("dcOmni270");
        omni0 = hardwareMap.dcMotor.get("dcOmni0");

        colorFront = hardwareMap.colorSensor.get("colorFront");
        colorFront.enableLed(false);

        // This is the FitnessGRAM Pacer Test. Line up at the start.
        waitForStart();

        int start;

        launchL.setPower(LAUNCH_MOTOR_SPEED);
        launchR.setPower(LAUNCH_MOTOR_SPEED);
        sleep(500);

        launchFlap.setPosition(FLAP_UP_POSITION);
        sleep(400);
        launchFlap.setPosition(0);
        sleep(1000);

        launchFlap.setPosition(FLAP_UP_POSITION);
        sleep(400);


        launchFlap.setPosition(0);
        launchL.setPower(0);
        launchR.setPower(0);

        EssentialHeading target;
        double gain = 0.008;

        target = new EssentialHeading(-40);
        start = omni90.getCurrentPosition();
        while (!(omni90.getCurrentPosition() < start - 6350) && opModeIsActive()) {
            fruity.driveWithRamper(target, -0.5, fruity.getNecessaryRotationPower(new EssentialHeading(0), gain));
        }
        fruity.drive(new EssentialHeading(0), 0, 0);

        target = new EssentialHeading(-90);
        while (!fruity.isFacing(target) && opModeIsActive()) {
            fruity.driveWithRamper(target, 0, fruity.getNecessaryRotationPower(target, gain));
        }
        fruity.drive(new EssentialHeading(0), 0, 0);

        start = omni90.getCurrentPosition();
        while (!(omni90.getCurrentPosition() < start - 1100) && opModeIsActive()) {
            fruity.driveWithRamper(target, -0.2, fruity.getNecessaryRotationPower(target, gain));
        }
        fruity.drive(new EssentialHeading(0), 0, 0);


        int leftDriveDistance = doPush();

        telemetry.addData("leftDriveDistance", leftDriveDistance);
        telemetry.update();

        target = new EssentialHeading(180);
        start = omni0.getCurrentPosition();
        while (!(omni0.getCurrentPosition() < start - leftDriveDistance) && opModeIsActive()) {
            fruity.driveWithRamper(target, 0.4, fruity.getNecessaryRotationPower(new EssentialHeading(-90), gain));
            telemetry.addData("omni0 pos", omni0.getCurrentPosition());
            telemetry.update();
        }
        fruity.drive(new EssentialHeading(0), 0, 0);

        /** // ----------------------------
         gain = 0.008;

         colorFront.enableLed(false);
         if (colorFront.blue() > colorFront.red()) {
         // good, slide left a tad
         target = new EssentialHeading(0);
         start = omni0.getCurrentPosition();
         while (!(omni0.getCurrentPosition() > start + 300) && opModeIsActive()) {
         fruity.driveWithRamper(target, -0.1, fruity.getNecessaryRotationPower(new EssentialHeading(90), gain));
         }
         fruity.drive(new EssentialHeading(0), 0, 0);
         leftDriveDistance = 5150;
         } else {
         // not good, slide right a tad
         target = new EssentialHeading(180);
         start = omni0.getCurrentPosition();
         while (!(omni0.getCurrentPosition() < start - 300) && opModeIsActive()) {
         fruity.driveWithRamper(target, 0.1, fruity.getNecessaryRotationPower(new EssentialHeading(90), gain));
         }
         fruity.drive(new EssentialHeading(0), 0, 0);
         leftDriveDistance = 5750;
         }

         // bump it
         // forward
         target = new EssentialHeading(90);
         start = omni90.getCurrentPosition();
         while (!(omni90.getCurrentPosition() > start + 400) && opModeIsActive()) {
         fruity.driveWithRamper(target, 0.1, fruity.getNecessaryRotationPower(new EssentialHeading(90), gain));
         }
         fruity.drive(new EssentialHeading(0), 0, 0);

         // reverse
         target = new EssentialHeading(-90);
         start = omni90.getCurrentPosition();
         while (!(omni90.getCurrentPosition() < start - 400) && opModeIsActive()) {
         fruity.driveWithRamper(target, 0.1, fruity.getNecessaryRotationPower(new EssentialHeading(90), gain));
         }
         fruity.drive(new EssentialHeading(0), 0, 0); **/
        doPush();

        target = new EssentialHeading(-50);
        start = omni90.getCurrentPosition();
        while (!(omni90.getCurrentPosition() > start + 4500) && opModeIsActive()) {
            fruity.driveWithRamper(target, 0.9, fruity.getNecessaryRotationPower(new EssentialHeading(-90), gain));
        }

        collector.setPower(-1);

        target = new EssentialHeading(0);
        start = omni0.getCurrentPosition();
        while (!(omni0.getCurrentPosition() > start + 3200) && opModeIsActive()) {
            fruity.driveWithRamper(target, 0.9, fruity.getNecessaryRotationPower(new EssentialHeading(-90), gain));
        }
        fruity.drive(new EssentialHeading(0), 0, 0);
    }

    int doPush() {

        EssentialHeading target;
        double gain = 0.008;
        int leftDriveDistance;
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
            leftDriveDistance = 5200;
        } else {
            // not good, slide right a tad
            target = new EssentialHeading(0);
            start = omni0.getCurrentPosition();
            while (!(omni0.getCurrentPosition() > start + 300) && opModeIsActive()) {
                fruity.driveWithRamper(target, 0.1, fruity.getNecessaryRotationPower(new EssentialHeading(-90), gain));
            }
            fruity.drive(new EssentialHeading(0), 0, 0);
            leftDriveDistance = 5800;
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


        return leftDriveDistance;
    }


}
