package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class OtosDrive {
    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.03;   // 0.02 Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.15;   // 0.015 Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.03;   // 0.01 Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 1;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 1;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 1;   //  Clip the turn speed to this max value (adjust for your robot)

    HardwareMap hardwareMap;
    Telemetry telemetry;
    private DcMotor leftFrontMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor leftRearMotor = null;
    private DcMotor rightRearMotor = null;

    static final double SLOW_SPEED = 0.1;
    static final double NORMAL_SPEED = 0.6;
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

    private ElapsedTime runtime = new ElapsedTime();
    // Sensors
    private SparkFunOTOS myOtos;        // Optical tracking odometry sensor
    SparkFunOTOS.Pose2D pos;

    public OtosDrive(HardwareMap hardwareMap, Telemetry telemetry) {

        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

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

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Mecanum drive is controlled with three axes: drive (front-and-back),
        // strafe (left-and-right), and twist (rotating the whole chassis).

        // Get a reference to the sensor
        myOtos = hardwareMap.get(SparkFunOTOS.class, "SparkFunOTOS");

    }
    public void configureOtos() {
        /*
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();
         */

        // Set the desired units for linear and angular measurements. Can be either
        // meters or inches for linear, and radians or degrees for angular. If not
        // set, the default is inches and degrees. Note that this setting is not
        // stored in the sensor, it's part of the library, so you need to set at the
        // start of all your programs.
        // myOtos.setLinearUnit(DistanceUnit.METER);
        myOtos.setLinearUnit(DistanceUnit.INCH);
        // myOtos.setAngularUnit(AnguleUnit.RADIANS);
        myOtos.setAngularUnit(AngleUnit.DEGREES);

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(.315, -5.125, 0); // should be -3.75 & -7.5 and 90
        myOtos.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        myOtos.setLinearScalar(1.000);
        myOtos.setAngularScalar(0.992);

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your programs. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total
        myOtos.calibrateImu();

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        myOtos.resetTracking();

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);
/*
        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.addLine();
        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
        telemetry.update();
 */
    }
    void setStartPosition(double currentX, double currentY, double currentHeading ) {
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(currentX, currentY, currentHeading);
        myOtos.setPosition(currentPosition);
    }
   public void startingPosition(SparkFunOTOS.Pose2D pose2D){
        myOtos.setPosition(pose2D);
    }

    /**
     * Move robot to a designated X,Y position and heading
     * set the maxTime to have the driving logic timeout after a number of seconds.
     */
    public void driveOtos(double targetX, double targetY, double targetHeading, int maxTime) {
        double drive, strafe, turn;
        double currentRange, targetRange, initialBearing, targetBearing, xError, yError, yawError;
        double opp, adj;

        SparkFunOTOS.Pose2D currentPos = myPosition();
        xError = targetX-currentPos.x;
        yError = targetY-currentPos.y;
        yawError = targetHeading-currentPos.h;

        runtime.reset();

        //while(opModeIsActive() && (runtime.milliseconds() < maxTime*1000) &&
        while(  (runtime.milliseconds() < maxTime*1000) &&
                ((Math.abs(xError) > 1) || (Math.abs(yError) > 1) || (Math.abs(yawError) > 2)) ) {
            double currentYawRadians = currentPos.h*3.1415/180;
            double rotX = xError * Math.cos(currentYawRadians) - yError * Math.sin(currentYawRadians);
            double rotY = xError * Math.sin(currentYawRadians) + yError * Math.cos(currentYawRadians);

            // Use the speed and turn "gains" to calculate how we want the robot to move.
//            drive  = Range.clip(yError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
//            strafe = Range.clip(xError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
//            turn   = Range.clip(yawError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
            drive  = Range.clip(rotY * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            strafe = Range.clip(rotX * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            turn   = Range.clip(yawError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;

            telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            // current x,y swapped due to 90 degree rotation
            telemetry.addData("current X coordinate", currentPos.x);
            telemetry.addData("current Y coordinate", currentPos.y);
            telemetry.addData("current Heading angle", currentPos.h);
            telemetry.addData("target X coordinate", targetX);
            telemetry.addData("target Y coordinate", targetY);
            telemetry.addData("target Heading angle", targetHeading);
            telemetry.addData("xError", xError);
            telemetry.addData("yError", yError);
            telemetry.addData("yawError", yawError);
            telemetry.update();

            // Apply desired axes motions to the drivetrain.
            moveRobot(drive, strafe, turn);

            // then recalc error
            currentPos = myPosition();
            xError = targetX-currentPos.x;
            yError = targetY-currentPos.y;
            yawError = targetHeading-currentPos.h;
        }
        moveRobot(0,0,0);
        currentPos = myPosition();
        /*
        telemetry.addData("current X c oordinate", currentPos.x);
        telemetry.addData("current Y coordinate", currentPos.y);
        telemetry.addData("current Heading angle", currentPos.h);
        telemetry.update();
         */
    }

    /* the reported OTOS values are based on sensor orientation, convert to robot centric
        by swapping x and y and changing the sign of the heading
        */
    public SparkFunOTOS.Pose2D myPosition() {
        pos = myOtos.getPosition();
        SparkFunOTOS.Pose2D myPos = new SparkFunOTOS.Pose2D(-pos.x, -pos.y, -pos.h);
        return(myPos);
    }

    /**
     * Move robot according to desired axes motions assuming robot centric point of view
     * Positive drive is forward
     * Positive strafe right
     * Positive turn is clockwise: note this is not how the IMU reports yaw(heading)
     */
    private void moveRobot(double drive, double strafe, double turn) {

        // Calculate wheel powers.
        double leftFrontPower    =  drive +strafe +turn;
        double rightFrontPower   =  drive -strafe -turn;
        double leftBackPower     =  drive -strafe +turn;
        double rightBackPower    =  drive +strafe -turn;

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
/*
        telemetry.addData("drivez", drive);
        telemetry.addData("leftFrontPower", leftFrontMotor.getPower());
        telemetry.addData("rightFrontPower", rightFrontMotor.getPower());
        telemetry.addData("leftBackPower", leftRearMotor.getPower());
        telemetry.addData("rightBackPower", rightRearMotor.getPower());
        telemetry.update();
        //sleep(10);

 */
    }
}
