package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by aedan on 12/17/16.
 */

@TeleOp(name="LaunchPowerTweak")
public class LaunchPowerTweak extends OpMode {


    final int LAUNCH_MS_TO_WAITING = 1000; // time until waiting state can be reached (e.g. flipper fall time)
    int LAUNCH_MS_TO_WAITING_LEFT = 0;
    final int LAUNCH_STATE_WAITING = 0; // wheels off, flipper down

    final int LAUNCH_STATE_STARTING = 1; // wheels accelerating, flipper down

    final int LAUNCH_MS_TO_FLIPPING = 500; // time left until flipping state can be reached (e.g wheel accelerate time)
    int LAUNCH_MS_TO_FLIPPING_LEFT = 0;
    final int LAUNCH_STATE_FLIPPING = 2; // wheels on, flipper rising

    final int LAUNCH_MS_TO_RETRACTING = 400; // time left until retracting state can be reached (e.g. flipper rise time)
    int LAUNCH_MS_TO_RETRACTING_LEFT = 0;
    final int LAUNCH_STATE_RETRACTING = 3; // wheels on, flipper retracting

    final double FLAP_UP_POSITION = 0.5;
    double LAUNCH_MOTOR_SPEED = 1;

    int launcherState = LAUNCH_STATE_RETRACTING;

    Servo launchFlap;

    DcMotor launchL;
    DcMotor launchR;

    boolean plast;

    public void init() {
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

        launchFlap.setPosition(0);

        plast = false;


    }
    private long lastMillis = System.currentTimeMillis();
    public void loop() {

        if (gamepad2.dpad_up && !plast) {
            LAUNCH_MOTOR_SPEED += 0.01; plast = true;
        }
        else if (gamepad2.dpad_down && !plast) {
            LAUNCH_MOTOR_SPEED -= 0.01; plast = true;
        }

        if (!gamepad2.dpad_down && !gamepad2.dpad_up){
            plast = false;
        }

        launchStateMachine(gamepad2.right_bumper);
        telemetry.addData("Speed:",LAUNCH_MOTOR_SPEED);
        telemetry.update();
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

}
