package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Kittay's Soft Serv-O")
@Disabled

public class Smacker extends OpMode {

    Servo schmack;

    public void init() {
        schmack = hardwareMap.servo.get("servoLoader");
    }

    public void loop() {
        schmack.setPosition(0);
        while (gamepad1.a) {try{Thread.sleep(1);}catch(InterruptedException e){}}
        telemetry.addData("Press de A butto to go boom","on de gamepad 1");
        while (!gamepad1.a) {try{Thread.sleep(1);}catch(InterruptedException e){}}
        schmack.setPosition(0.5);
        try{Thread.sleep(200);}catch(InterruptedException e){}
    }

}
