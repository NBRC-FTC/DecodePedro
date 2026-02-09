
package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.IntakeWheel;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.LauncherWheel;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

//

@Autonomous(name = "Pedro Autonomous 9 Balls", group = "Autonomous")
@Configurable // Panels
public class PedroAutonomous9Ball extends OpMode {
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
        public PathChain ScorePreload;
        public PathChain Grab1;
        public PathChain Score1;
        public PathChain Grab2;
        public PathChain Score2;

        public Paths(Follower follower) {
            ScorePreload = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(23.000, 125.000),

                                    new Pose(34.000, 117.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(145))

                    .build();

            Grab1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(34.000, 117.000),
                                    new Pose(94.000, 83.000),
                                    new Pose(22, 84.000)
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            Score1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(18.000, 84.000),

                                    new Pose(34.000, 117.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-1), Math.toRadians(145))

                    .build();

            Grab2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(34.000, 117.000),
                                    new Pose(108.000, 57.000),
                                    new Pose(11.000, 58.000)
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            Score2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(11.000, 58.000),
                                    new Pose(35.000, 51.000),
                                    new Pose(34.000, 117.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(145))

                    .build();
        }
    }



    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.ScorePreload,.5,true);
                setPathState(1);
                break;
            case 1:
                //Shoot
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 5) {
                        follower.followPath(paths.Grab1,.5,true);
                        setPathState(2);
                    }
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Score1,.5,true);
                    setPathState(3);
                }
                break;
            case 3:
                //Shoot
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 5) {
                        follower.followPath(paths.Grab2,.5,true);
                        setPathState(4);
                    }
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Score2,.5,true);
                }
                break;
        }
    }
    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}
    