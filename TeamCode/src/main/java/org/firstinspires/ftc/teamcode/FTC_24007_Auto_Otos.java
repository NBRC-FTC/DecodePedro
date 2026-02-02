package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.IntakeWheel;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.LauncherWheel;
import org.firstinspires.ftc.teamcode.subsystems.OtosDrive;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;


@Autonomous(name="AutoMode_Otos",preselectTeleOp = "TeleOpMode")  // @TeleOp(...) is the other common choice
// @Disabled
public class FTC_24007_Auto_Otos extends LinearOpMode {

    private  LauncherWheel launcherWheel;
    private  IntakeWheel intakeWheel;
    private Launcher launcher;
    private Shooter shooter;
    //private Mecanum mecanum;
    public enum START_POSITION{
        GOAL_LAUNCH_RED,
        SMALL_LAUNCH_RED,
        SMALL_LAUNCH_BLUE,
        GOAL_LAUNCH_BLUE,
        SMALL_PARK,
    }

    public static START_POSITION startPosition;

    @Override
    public void runOpMode() {
        telemetry.setAutoClear(true);
        OtosDrive otosDrive = new OtosDrive(hardwareMap, telemetry);
        otosDrive.configureOtos();
        launcherWheel = new LauncherWheel(hardwareMap, telemetry);
        intakeWheel = new IntakeWheel(hardwareMap);
        launcher = new Launcher(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry);
        //mecanum = new Mecanum(hardwareMap);


        telemetry.addData("Configured OTOS", startPosition);
        telemetry.update();



        //Key Pay inputs to selecting Starting Position of robot
        selectStartingPosition();
        telemetry.addData("Selected Starting Position", startPosition);
        telemetry.update();

        int delay = 0;
        delay = setStartDelay();

        telemetry.addData("Selected Starting delay", delay);
        telemetry.update();

        while (opModeInInit()) {
            telemetry.addData("Selected Starting Position", startPosition);
            telemetry.addData("Selected Starting delay", delay);
            // Display current robot position/heading
            telemetry.addData("Current X coordinate", otosDrive.myPosition().x);
            telemetry.addData("Current Y coordinate", otosDrive.myPosition().y);
            telemetry.addData("Current Heading angle", otosDrive.myPosition().h);
            telemetry.update();
        }

        // Wait for the game to start (driver presses PLAY)

        waitForStart();
/*
        telemetry.addLine("hello");
        telemetry.update();
        otosDrive.driveOtos(0, -36, 0, 5);
        sleep(5000);
        otosDrive.driveOtos(-10, -36, 0, 5);
        sleep(20000);
        //otosDrive.moveRobot(0, 0, 0);
        sleep(20000);
        //otosDrive.moveRobot(0, 0, 0);
*/

        switch (startPosition) {
            case GOAL_LAUNCH_BLUE:
//                SparkFunOTOS.Pose2D pose2D = new SparkFunOTOS.Pose2D(-55.79, 56.504867, 35.954000); // should be -3.75 & -7.5 and 90
//                otosDrive.startingPosition(pose2D);
                telemetry.addData("Current X coordinate", otosDrive.myPosition().x);
                telemetry.addData("Current Y coordinate", otosDrive.myPosition().y);
                telemetry.addData("Current Heading angle", otosDrive.myPosition().h);
                telemetry.update();
                sleep(delay);
                shooter.shootNear();
                otosDrive.driveOtos(0, -22, 0, 10);
                telemetry.addData("Current X coordinate", otosDrive.myPosition().x);
                telemetry.addData("Current Y coordinate", otosDrive.myPosition().y);
                telemetry.addData("Current Heading angle", otosDrive.myPosition().h);
                telemetry.update();
                sleep(500);
//                sleep(2000);
//                launcher.shooterUp();
//                sleep(1000);
//                launcher.shooterDown();
//                sleep(500);
//                shooter.shootNear();
//                sleep(2000);
//                launcher.shooterUp();
//                sleep(1000);
//                launcher.shooterDown();
//                sleep(500);
//                shooter.shootNear();
//                sleep(2000);
//                launcher.shooterUp();
//                sleep(1000);
//                launcher.shooterDown();
//                sleep(500);
//                shooter.shootNear();
//                sleep(2000);
//                launcher.shooterUp();
//                sleep(1000);
                launcherWheel.LauncherOn();
                intakeWheel.IntakeOn();
                sleep(2500);
                launcherWheel.LauncherOff();
                otosDrive.driveOtos(0,-63,305,10);

//                sleep(5000);
//                launcherWheel.LauncherOff();
//                intakeWheel.IntakeSpit();
//                sleep(200);
//                launcherWheel.LauncherOn();
//                intakeWheel.IntakeOn();
//                sleep(5000);
//                intakeWheel.IntakeOff();
//                launcherWheel.LauncherOff();
//                otosDrive.driveOtos(-15, -22, 0, 10);
//                break;

            case SMALL_PARK:
               sleep(delay);
                otosDrive.driveOtos(0,25,0,10);
                break;
            case SMALL_LAUNCH_BLUE:
               sleep(delay);
                otosDrive.driveOtos(-5,50,0,10);
                sleep(1000);
                otosDrive.driveOtos(-16,75,-52,10);
                shooter.shootNear();
                sleep(2000);
                launcher.shooterUp();
                sleep(1000);
                launcher.shooterDown();
                sleep(2000);
                launcher.shooterUp();
                sleep(1000);
                otosDrive.driveOtos(-16, 71, -52, 10);
                sleep(2000);
                launcher.shooterDown();
                sleep(2000);
                launcher.shooterUp();
                sleep(1000);
                launcher.shooterDown();
                sleep(2000);
                launcher.shooterUp();
                sleep(2000);
                otosDrive.driveOtos(-15, 35, -52, 10);
                break;
            case SMALL_LAUNCH_RED:
               sleep(delay);
                otosDrive.driveOtos(5,50,0,10);
                sleep(1000);
                otosDrive.driveOtos(16,75,52,10);
                shooter.shootNear();
                sleep(2000);
                launcher.shooterUp();
                sleep(1000);
                launcher.shooterDown();
                sleep(2000);
                launcher.shooterUp();
                sleep(1000);
                otosDrive.driveOtos(16, 71, 52, 10);
                sleep(2000);
                launcher.shooterDown();
                sleep(2000);
                launcher.shooterUp();
                sleep(1000);
                launcher.shooterDown();
                sleep(2000);
                launcher.shooterUp();
                sleep(2000);
                otosDrive.driveOtos(15, 35, 52, 10);
                break;
            case GOAL_LAUNCH_RED:
                //SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(.0, 0, 0); // should be -3.75 & -7.5 and 90
                //otosDrive.setPosition(currentPosition);
                telemetry.addData("Current X coordinate", otosDrive.myPosition().x);
                telemetry.addData("Current Y coordinate", otosDrive.myPosition().y);
                telemetry.addData("Current Heading angle", otosDrive.myPosition().h);
                telemetry.update();
                sleep(delay);
                otosDrive.driveOtos(0, -22, 0, 10);
                telemetry.addData("Current X coordinate", otosDrive.myPosition().x);
                telemetry.addData("Current Y coordinate", otosDrive.myPosition().y);
                telemetry.addData("Current Heading angle", otosDrive.myPosition().h);
                telemetry.update();
                sleep(500);
                shooter.shootNear();
                sleep(2000);
                launcher.shooterUp();
                sleep(1000);
                launcher.shooterDown();
                sleep(500);
                shooter.shootNear();
                sleep(2000);
                launcher.shooterUp();
                sleep(1000);
                launcher.shooterDown();
                sleep(500);
                shooter.shootNear();
                sleep(2000);
                launcher.shooterUp();
                sleep(1000);
                launcher.shooterDown();
                sleep(500);
                shooter.shootNear();
                sleep(2000);
                launcher.shooterUp();
                sleep(1000);
                otosDrive.driveOtos(26, -22, 0, 10);
                break;



        }
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // Pause to display last telemetry message.
    }
    public void selectStartingPosition() {
        telemetry.setAutoClear(true);
        telemetry.clearAll();
        //******select start pose*****
        while(!isStopRequested()){
            //telemetry.addData("Initializing FTC Wires (ftcwires.org) Autonomous adopted for Team:",
            //       TEAM_NAME, " ", TEAM_NUMBER);
            //telemetry.addData("---------------------------------------","");
            //telemetry.addLine("This Auto program uses Open CV Vision Processor for Team Element detection");
            telemetry.addData("Select Starting Position:","");
            telemetry.addData("    Goal Launch Red  ", "(X / Square)");
            telemetry.addData("    Small Launch_BLUE ", "(Y / Triangle)");
            telemetry.addData("    Small Launch_RED ", "(Right Bumper)");
            telemetry.addData("    Goal Launch Blue  ", "(B / Circle)");
            telemetry.addData("    Small Park ", "(A / Cross)");


            if(gamepad1.x){
                startPosition = START_POSITION.GOAL_LAUNCH_RED;
                break;
            }
            if(gamepad1.y){
                startPosition = START_POSITION.SMALL_LAUNCH_BLUE;
                break;
            }
            if(gamepad1.right_bumper) {
                startPosition = START_POSITION.SMALL_LAUNCH_RED;
                break;
            }
            if(gamepad1.b){
                startPosition = START_POSITION.GOAL_LAUNCH_BLUE;
                break;
            }
            if(gamepad1.a){
                startPosition = START_POSITION.SMALL_PARK;

                break;
            }
            telemetry.update();
        }
        telemetry.clearAll();
    }

    public int setStartDelay() {
        telemetry.setAutoClear(true);
        telemetry.clearAll();
        int delay = 0;

        //******select start pose*****
        while(!isStopRequested()){
            //telemetry.addData("Initializing FTC Wires (ftcwires.org) Autonomous adopted for Team:",
            //       TEAM_NAME, " ", TEAM_NUMBER);
            //telemetry.addData("---------------------------------------","");
            //telemetry.addLine("This Auto program uses Open CV Vision Processor for Team Element detection");
            telemetry.addData("Select Starting Delay ","");
            telemetry.addData("    Increase starting delay Park ", "(dpad up)");
            telemetry.addData("    Deacrease starting delay ", "(dpad down)");
            telemetry.addData("delay", delay);
            telemetry.addData("   Lock in set delay ", "(right stick button)");

            if(gamepad1.dpadUpWasPressed()){
                delay = delay + 1000;
            }
            if(gamepad1.dpadDownWasPressed()){
                delay = delay - 1000;
                if (delay< 0){
                    delay = 0;
                }
            }
            if(gamepad1.right_stick_button) {
                break;
            }

            telemetry.update();
        }
        telemetry.clearAll();
            return delay;
    }

    public void displayPosition(OtosDrive otosDrive) {
        telemetry.addData("Current X coordinate", otosDrive.myPosition().x);
        telemetry.addData("Current Y coordinate", otosDrive.myPosition().y);
        telemetry.addData("Current Heading angle", otosDrive.myPosition().h);
        telemetry.update();
    }
}
