package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LauncherWheel {
    HardwareMap hardwareMap;

    private DcMotor launcherMotor = null;

    public LauncherWheel(HardwareMap hardwareMap, Telemetry telemetry) {

        this.hardwareMap = hardwareMap;
        launcherMotor = hardwareMap.get(DcMotor.class, "launcher_motor");
        launcherMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void LauncherOn(){
        launcherMotor.setPower(1);
    }

    public void LauncherOff(){
        launcherMotor.setPower(0);
    }

    public void LauncherSpit(){
        launcherMotor.setPower(-0.3);
    }

    public void setLauncherSpeed (float speed){
        launcherMotor.setPower(speed);
    }
}
