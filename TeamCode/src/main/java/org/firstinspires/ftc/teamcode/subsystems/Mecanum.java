package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Mecanum {
    HardwareMap hardwareMap;

    private DcMotor leftFrontMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor leftRearMotor = null;
    private DcMotor rightRearMotor = null;

    static final double SLOW_SPEED = .3;
    static final double NORMAL_SPEED = .6;
    static final double FAST_SPEED = 1;
    static final double CLICKS_PER_INCH = 87.5;
    static final double CLICKS_PER_DEGRE = 21.94;

    private int lfPos;
    private int rfPos;
    private int lrPos;
    private int rrPos;

    public enum SPEED {
        SLOW,
        NORMAL,
        FAST
    }
    // This declares the IMU needed to get the current direction the robot is facing
    IMU imu;

    public Mecanum(HardwareMap hardwareMap) {

        this.hardwareMap = hardwareMap;
        leftFrontMotor = hardwareMap.get(DcMotor.class, "front_left");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "front_right");
        leftRearMotor = hardwareMap.get(DcMotor.class, "back_left");
        rightRearMotor = hardwareMap.get(DcMotor.class, "back_right");

        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotor.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set the drive motor run modes:
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Mecanum drive is controlled with three axes: drive (front-and-back),
        // strafe (left-and-right), and twist (rotating the whole chassis).
        imu = hardwareMap.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

    }

    public void driveMecanum(double drive, double strafe, double twist, SPEED speed) {
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double speedMultiplier;

        switch (speed) {
            case SLOW:
                speedMultiplier = SLOW_SPEED;
                break;
            case FAST:
                speedMultiplier = FAST_SPEED;
                break;
            default:
                speedMultiplier = NORMAL_SPEED;
        }

        // Calculate wheel powers.
        double leftFrontPower    =  (drive +strafe +twist) * speedMultiplier;
        double rightFrontPower   =  (drive -strafe -twist) * speedMultiplier;
        double leftBackPower     =  (drive -strafe +twist) * speedMultiplier;
        double rightBackPower    =  (drive +strafe -twist) * speedMultiplier;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        leftFrontMotor.setPower(leftFrontPower);
        rightFrontMotor.setPower(rightFrontPower);
        leftRearMotor.setPower(leftBackPower);
        rightRearMotor.setPower(rightBackPower);
    }

    // This routine drives the robot field relative
    public void driveFieldRelative(double forward, double right, double rotate, SPEED speed) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        driveMecanum(newForward, newRight, rotate, speed);
    }

    public void resetImu(){
        imu.resetYaw();
    }
    public double getImuAngle() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void moveForward(int howMuch, double speed) {
        // howMuch is in inches. A negative howMuch moves backward.

        // fetch motor positions
        lfPos = leftFrontMotor.getCurrentPosition();
        rfPos = rightFrontMotor.getCurrentPosition();
        lrPos = leftRearMotor.getCurrentPosition();
        rrPos = rightRearMotor.getCurrentPosition();

        // calculate new targets
        lfPos += howMuch * CLICKS_PER_INCH;
        rfPos += howMuch * CLICKS_PER_INCH;
        lrPos += howMuch * CLICKS_PER_INCH;
        rrPos += howMuch * CLICKS_PER_INCH;

        // move robot to new position
        leftFrontMotor.setTargetPosition(lfPos);
        rightFrontMotor.setTargetPosition(rfPos);
        leftRearMotor.setTargetPosition(lrPos);
        rightRearMotor.setTargetPosition(rrPos);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontMotor.setPower(speed);
        rightFrontMotor.setPower(speed);
        leftRearMotor.setPower(speed);
        rightRearMotor.setPower(speed);

        // wait for move to complete
        while (leftFrontMotor.isBusy() && rightFrontMotor.isBusy() &&
                leftRearMotor.isBusy() && rightRearMotor.isBusy()) {

            // Display it for the driver.
            //telemetry.addLine("Move Foward");
            //telemetry.addData("Target", "%7d :%7d", lfPos, rfPos, lrPos, rrPos);
            //telemetry.addData("Actual", "%7d :%7d", leftFrontMotor.getCurrentPosition(),
            //    rightFrontMotor.getCurrentPosition(), leftRearMotor.getCurrentPosition(),
            //      rightRearMotor.getCurrentPosition());
            //telemetry.update();
        }
        // Stop all motion;
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftRearMotor.setPower(0);
        rightRearMotor.setPower(0);
    }
    public void moveRight(int howMuch, double speed) {
        // howMuch is in inches. A negative howMuch moves backward.

        // fetch motor positions
        lfPos = leftFrontMotor.getCurrentPosition();
        rfPos = rightFrontMotor.getCurrentPosition();
        lrPos = leftRearMotor.getCurrentPosition();
        rrPos = rightRearMotor.getCurrentPosition();

        // calculate new targets
        lfPos += howMuch * CLICKS_PER_INCH;
        rfPos -= howMuch * CLICKS_PER_INCH;
        lrPos -= howMuch * CLICKS_PER_INCH;
        rrPos += howMuch * CLICKS_PER_INCH;

        // move robot to new position
        leftFrontMotor.setTargetPosition(lfPos);
        rightFrontMotor.setTargetPosition(rfPos);
        leftRearMotor.setTargetPosition(lrPos);
        rightRearMotor.setTargetPosition(rrPos);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontMotor.setPower(speed);
        rightFrontMotor.setPower(speed);
        leftRearMotor.setPower(speed);
        rightRearMotor.setPower(speed);

        // wait for move to complete
        while (leftFrontMotor.isBusy() && rightFrontMotor.isBusy() &&
                leftRearMotor.isBusy() && rightRearMotor.isBusy()) {

            // Display it for the driver.
            //telemetry.addLine("Strafe Right");
            //telemetry.addData("Target", "%7d :%7d", lfPos, rfPos, lrPos, rrPos);
            //telemetry.addData("Actual", "%7d :%7d", leftFrontMotor.getCurrentPosition(),
              //      rightFrontMotor.getCurrentPosition(), leftRearMotor.getCurrentPosition(),
                //    rightRearMotor.getCurrentPosition());
            //telemetry.update();
        }

        // Stop all motion;
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftRearMotor.setPower(0);
        rightRearMotor.setPower(0);

    }
    public void turnLeft(int howMuch, double speed) {
        // howMuch is in inches. A negative howMuch moves backward.

        // fetch motor positions
        lfPos = leftFrontMotor.getCurrentPosition();
        rfPos = rightFrontMotor.getCurrentPosition();
        lrPos = leftRearMotor.getCurrentPosition();
        rrPos = rightRearMotor.getCurrentPosition();

        // calculate new targets
        lfPos += -howMuch * CLICKS_PER_INCH;
        rfPos += howMuch * CLICKS_PER_INCH;
        lrPos += -howMuch * CLICKS_PER_INCH;
        rrPos += howMuch * CLICKS_PER_INCH;

        // move robot to new position
        leftFrontMotor.setTargetPosition(lfPos);
        rightFrontMotor.setTargetPosition(rfPos);
        leftRearMotor.setTargetPosition(lrPos);
        rightRearMotor.setTargetPosition(rrPos);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontMotor.setPower(speed);
        rightFrontMotor.setPower(speed);
        leftRearMotor.setPower(speed);
        rightRearMotor.setPower(speed);

        // wait for move to complete
        while (leftFrontMotor.isBusy() && rightFrontMotor.isBusy() &&
                leftRearMotor.isBusy() && rightRearMotor.isBusy()) {

            // Display it for the driver.
            //telemetry.addLine("Move Foward");
            //telemetry.addData("Target", "%7d :%7d", lfPos, rfPos, lrPos, rrPos);
            //telemetry.addData("Actual", "%7d :%7d", leftFrontMotor.getCurrentPosition(),
            //    rightFrontMotor.getCurrentPosition(), leftRearMotor.getCurrentPosition(),
            //      rightRearMotor.getCurrentPosition());
            //telemetry.update();
        }
        // Stop all motion;
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftRearMotor.setPower(0);
        rightRearMotor.setPower(0);
    }
}