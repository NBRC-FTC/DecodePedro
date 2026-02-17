/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.subsystems.IntakeWheel;
import org.firstinspires.ftc.teamcode.subsystems.LauncherWheel;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name="TeleOpMode")

public class FTC_24007_TeleOp_101225 extends LinearOpMode {
    // Declare OpMode members.
    private double DESIRED_DISTANCE = 37; //  this is how close the camera should get to the target (inches)

    private ElapsedTime runtime = new ElapsedTime();

    private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    final double SPEED_GAIN  =  0.2; //0.02  ;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.1;//0.015 ;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    final double TURN_GAIN   =  0.15;//0.01  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the strafing speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    //private Launcher launcher;
    private LauncherWheel launcherwheel;
    private Shooter shooter;
    private Mecanum mecanum;
    private IntakeWheel intakeWheel;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

       // launcher = new Launcher(hardwareMap, telemetry);
        launcherwheel = new LauncherWheel(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry);
        mecanum = new Mecanum(hardwareMap);
        intakeWheel = new IntakeWheel(hardwareMap);
        Mecanum.SPEED speed;

        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;


        // Initialize the Apriltag Detection process
        initAprilTag();
        setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad1.right_bumper){
                speed = Mecanum.SPEED.FAST;
            } else if (gamepad1.left_bumper) {
                speed = Mecanum.SPEED.SLOW;
            } else {
                speed = Mecanum.SPEED.NORMAL;
            }

            targetFound = false;
            desiredTag  = null;

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }
            if(targetFound){
                gamepad1.rumble(0,.2,100);
            }
            // Tell the driver what we see, and what to do.
            if (targetFound) {
                telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
            } else {
                telemetry.addData("\n>","Drive using joysticks to find valid target\n");
            }

            //mecanum.driveMecanum( -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, speed );

            if (gamepad1.a && targetFound) {

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double  headingError    = desiredTag.ftcPose.bearing;
                double  yawError        = desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn   = Range.clip(-headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                //strafe = Range.clip(yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                strafe = gamepad1.left_stick_x ; // 2.0;  // Reduce strafe rate to 50%.

                if (drive<.1 && turn<.1){
                    gamepad1.rumble(1,0,100);
                }

                telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
                mecanum.driveMecanum(drive, strafe, turn, speed);

            } else {

                // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
                drive  = -gamepad1.left_stick_y;  // 2.0;  // Reduce drive rate to 50%.
                strafe = gamepad1.left_stick_x ; // 2.0;  // Reduce strafe rate to 50%.
                turn   = gamepad1.right_stick_x; // 3.0;  // Reduce turn rate to 33%.
                telemetry.addData("Manual","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
                mecanum.driveFieldRelative(drive, strafe, turn, speed);
            }
            intakeWheel.setIntakeSpeed(gamepad1.right_trigger - gamepad1.left_trigger);

            if (gamepad1.psWasPressed()) {
                mecanum.resetImu();
            }
            if (gamepad1.xWasPressed()) {
                shooter.shootNear();
                DESIRED_DISTANCE = 26;
            }
            if (gamepad1.dpadLeftWasPressed()) {
                shooter.shooterStop();
            }
            if (gamepad1.bWasPressed()) {
                shooter.shootMed();
                DESIRED_DISTANCE = 46;
            }
            if (gamepad1.dpadUpWasPressed()){
                shooter.increaseVelocity();
            }
            if (gamepad1.dpadDownWasPressed()){
                shooter.decreaseVelocity();
            }
            /*
            if (gamepad1.leftStickButtonWasPressed()) {
                launcher.shoot();
            }
            */

            if (gamepad1.y
//                    && shooter.isShooterAtTargetVelocity()
            ) {
             launcherwheel.LauncherOn();
             intakeWheel.IntakeOn();
            }else{
                launcherwheel.LauncherOff();

            }
            // Show the elapsed game time and wheel power.
            // telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Game Pad 1", "Left stick y:" + gamepad1.left_stick_y);
            telemetry.addData("Game Pad 1", "Left stick x:" + gamepad1.left_stick_x);
            telemetry.addData("Game Pad 1", "Right stick x:" + gamepad1.right_stick_x);
//            telemetry.addData("Game Pad 2", "Right stick y:" + gamepad2.right_stick_y);
            telemetry.addData("Desired Distance", DESIRED_DISTANCE);
            telemetry.addData("Yaw", mecanum.getImuAngle());
            shooter.getShooterVelocity();
            telemetry.update();
        }
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();
        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // e.g. Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
}