package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Launcher {
    private Servo launcher;
    public Launcher(HardwareMap hardwareMapInit, Telemetry telemetry) {
        launcher = hardwareMapInit.get(Servo.class, "Launcher");
    }
    public void shooterUp() {
        launcher.setPosition(1);
    }
    public void shooterDown() {
        launcher.setPosition(0);
    }
    public void shoot() {
        if (launcher.getPosition() == 0) {
            launcher.setPosition(1);
        } else if (launcher.getPosition() == 1) {
            launcher.setPosition(0);
        }
    }
}