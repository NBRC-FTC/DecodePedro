package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeWheel {
    HardwareMap hardwareMap;

    private DcMotor intakeMotor = null;

    public IntakeWheel(HardwareMap hardwareMap) {

        this.hardwareMap = hardwareMap;
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    public void IntakeOn(){
        intakeMotor.setPower(0.7);

    }
    public void IntakeOff(){
        intakeMotor.setPower(0);

    }
    public void IntakeSpit(){
        intakeMotor.setPower(-0.3);
    }

    public void setIntakeSpeed (float speed){
        intakeMotor.setPower(speed);
    }
}
