package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter {
    private DcMotorEx leftShooterMotor;
    private DcMotorEx rightShooterMotor;
    private Telemetry telemetry;
    private double targetShooterVelocity = 0;
    public Shooter(HardwareMap hardwareMapInit, Telemetry telemetry) {
        leftShooterMotor = hardwareMapInit.get(DcMotorEx.class, "shooterLeft");
        rightShooterMotor = hardwareMapInit.get(DcMotorEx.class, "shooterRight");
        leftShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftShooterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightShooterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.telemetry = telemetry;

    }
    public boolean isShooterAtTargetVelocity(){
        if (targetShooterVelocity >= leftShooterMotor.getVelocity()-40 && targetShooterVelocity >= rightShooterMotor.getVelocity()-40 && targetShooterVelocity <= leftShooterMotor.getVelocity()+40 && targetShooterVelocity <= rightShooterMotor.getVelocity()+40){
            return true;
        }else{
            return false;
        }
    }
    public void setShooterVelocity(double velocity){
        leftShooterMotor.setVelocity(velocity);
        rightShooterMotor.setVelocity(velocity);
    }
    public void getShooterVelocity(){
        telemetry.addData("leftShooterVelocity", leftShooterMotor.getVelocity());
        telemetry.addData("rightShooterVelocity", rightShooterMotor.getVelocity());
    }
    public void shooterStop (){
        leftShooterMotor.setVelocity(0);
        rightShooterMotor.setVelocity(0);
        targetShooterVelocity = 0;
    }
    public void shootNear(){
        targetShooterVelocity = 740;
        setShooterVelocity(targetShooterVelocity); //26 inch
    }
    public void shootMed(){
        targetShooterVelocity = 1000; //46 inch
        setShooterVelocity(targetShooterVelocity);
    }
    public void increaseVelocity(){
        targetShooterVelocity = targetShooterVelocity+20;
        setShooterVelocity(targetShooterVelocity);
    }
    public void decreaseVelocity(){
        targetShooterVelocity = targetShooterVelocity-20;
        setShooterVelocity(targetShooterVelocity);
    }
}
