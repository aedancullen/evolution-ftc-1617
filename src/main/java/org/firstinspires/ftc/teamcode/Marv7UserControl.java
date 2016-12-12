package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;

import io.github.aedancullen.fruity.FruityController;
import io.github.aedancullen.fruity.MotorConfigurations;

@TeleOp(name="Marv Mk7 User Control", group="MarvMk7")
//@Disabled
public class Marv7UserControl extends OpMode {

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

    //Other hardware not used by Fruity
    DcMotor collector;
    DcMotor launchL;
    DcMotor launchR;
    Servo launchFlap;

    int launcherState = LAUNCH_STATE_RETRACTING;


    public void init() {
        fruity = new FruityController(hardwareMap, telemetry, "imu",
                Arrays.asList(
                        hardwareMap.dcMotor.get("dcOmni0"),
                        hardwareMap.dcMotor.get("dcOmni90"),
                        hardwareMap.dcMotor.get("dcOmni180"),
                        hardwareMap.dcMotor.get("dcOmni270")
                ),
                DcMotorSimple.Direction.REVERSE,
                DcMotor.RunMode.RUN_USING_ENCODER,
                DcMotor.ZeroPowerBehavior.FLOAT,
                MotorConfigurations.QUAD_NONDIAGONAL_SHORT,
                0.002, 0.01, 0.1);

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
    }

    private long lastMillis = System.currentTimeMillis();
    public void loop() {
        // Driver 1 (driving)
        fruity.updateHeadingNow();
        fruity.handleGamepad(gamepad1);

        // Driver 2 (accessories)
        if (gamepad2.a) {
            collector.setPower(1);
        }
        else if (gamepad2.b) {
            collector.setPower(-1);
        }
        else {
            collector.setPower(0);
        }

        // Run state machine
        launchStateMachine(gamepad2.right_bumper);
    }

    public void launchStateMachine(boolean trigger) {
        // Driver 2 Launcher State Machine ----------------------------------------------------------------------
        // AKA fun with spinning wheels and flipper thingies
        if (launcherState == LAUNCH_STATE_RETRACTING && LAUNCH_MS_TO_WAITING_LEFT <= 0) {
            // Transition automatically to stability at the next state
            if (trigger) {
                // Rapid fire - go directly to flipping
                launchFlap.setPosition(FLAP_UP_POSITION);
                launcherState = LAUNCH_STATE_FLIPPING;
                LAUNCH_MS_TO_RETRACTING_LEFT = LAUNCH_MS_TO_RETRACTING;
            }
            else {
                launchFlap.setPosition(0);
                launchR.setPower(0);
                launchL.setPower(0);
                launcherState = LAUNCH_STATE_WAITING;
            }
        }
        else if (launcherState == LAUNCH_STATE_WAITING && trigger) {
            // Transition from user input to instability
            launchL.setPower(LAUNCH_MOTOR_SPEED);
            launchR.setPower(LAUNCH_MOTOR_SPEED);
            launcherState = LAUNCH_STATE_STARTING;
            LAUNCH_MS_TO_FLIPPING_LEFT = LAUNCH_MS_TO_FLIPPING; // prepare to reach flipping state
        }
        else if (launcherState == LAUNCH_STATE_STARTING && LAUNCH_MS_TO_FLIPPING_LEFT <= 0) {
            // Transition automatically to instability in this case -- waiting for flipper to rise!
            launchFlap.setPosition(FLAP_UP_POSITION);
            launcherState = LAUNCH_STATE_FLIPPING;
            LAUNCH_MS_TO_RETRACTING_LEFT = LAUNCH_MS_TO_RETRACTING; // prepare to reach retracting state
        }
        else if (launcherState == LAUNCH_STATE_FLIPPING && LAUNCH_MS_TO_RETRACTING_LEFT <= 0) {
            launchFlap.setPosition(0);
            launcherState = LAUNCH_STATE_RETRACTING;
            LAUNCH_MS_TO_WAITING_LEFT = LAUNCH_MS_TO_WAITING;
        }
        //Timer update
        long elapsed = System.currentTimeMillis() - lastMillis;
        lastMillis = System.currentTimeMillis();
        if (launcherState == LAUNCH_STATE_STARTING) {
            LAUNCH_MS_TO_FLIPPING_LEFT -= elapsed;
        }
        else if (launcherState == LAUNCH_STATE_FLIPPING) {
            LAUNCH_MS_TO_RETRACTING_LEFT -= elapsed;
        }
        else if (launcherState == LAUNCH_STATE_RETRACTING) {
            LAUNCH_MS_TO_WAITING_LEFT -= elapsed;
        }
    }

    public void stop() {
        launchL.setPower(0);
        launchR.setPower(0);
        collector.setPower(0);
    }

}
