package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp(name="MaxSonar Test")
public class MaxSonarTest extends OpMode {

    AnalogInput sonar;

    public void init() {
        sonar = hardwareMap.analogInput.get("sonar");
    }

    public void loop() {
        telemetry.addData("De sensor say", (sonar.getVoltage() / 0.000977) + " mm");
    }

}
