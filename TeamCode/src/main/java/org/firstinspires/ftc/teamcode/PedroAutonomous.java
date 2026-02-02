
package org.firstinspires.ftc.teamcode;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.subsystems.IntakeWheel;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.LauncherWheel;
import org.firstinspires.ftc.teamcode.subsystems.OtosDrive;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

//

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class PedroAutonomous extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private Timer pathTimer, actionTimer, opmodeTimer;
    private  LauncherWheel launcherWheel;
    private  IntakeWheel intakeWheel;
    private Launcher launcher;
    private Shooter shooter;
    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(23, 125, Math.toRadians(145)));

        launcherWheel = new LauncherWheel(hardwareMap, telemetry);
        intakeWheel = new IntakeWheel(hardwareMap);
        launcher = new Launcher(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry);

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Total Time:", opmodeTimer.getElapsedTimeSeconds());
        panelsTelemetry.debug("Path Time:", pathTimer.getElapsedTimeSeconds());
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(23.000, 125.000),

                                    new Pose(34.000, 117.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(145))
                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(34.000, 117.000),

                                    new Pose(47.000, 127.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(145))

                    .build();
        }
    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                shooter.shootNear();
                setPathState(1);
                break;
            case 1:
                follower.followPath(paths.Path1,.25, true);  //MJR ToDo
                    setPathState(2);
                break;
            case 2:
                if(!follower.isBusy()){
                    intakeWheel.IntakeOn();
                    launcherWheel.LauncherOn();
                    if(pathTimer.getElapsedTimeSeconds() > 10){
                        intakeWheel.IntakeOff();
                        launcherWheel.LauncherOff();
                        setPathState(3);
                    }
                }
                break;
            case 3:
            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(paths.Path2,true);  //MJR ToDo
                    setPathState(4);
                }
                break;
            case 4:
                shooter.shooterStop();
                break;
        }
    }
    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}
    