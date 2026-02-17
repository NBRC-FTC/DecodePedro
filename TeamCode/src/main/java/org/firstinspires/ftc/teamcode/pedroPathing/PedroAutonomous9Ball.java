
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
        public PathChain Park;
        public final Pose startPose = new Pose(23, 125, Math.toRadians(145)); // Start Pose of our robot.
        public final Pose scorePose = new Pose(34, 117, Math.toRadians(145)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
        public final Pose pickup1Pose = new Pose(38, 86, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
        public final Pose pickup1front = new Pose(62,86,Math.toRadians(0)); // In Front of Highest (First Set) of Artifacts from the Spike Mark.
        public final Pose pickup2Pose = new Pose(33, 65, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
        public final Pose pickup2front = new Pose(62, 65, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
        public final Pose parkPose = new Pose(40, 125, Math.toRadians(145));
//        public final Pose pickup3Pose = new Pose(49, 135, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.
        public Paths(Follower follower) {
            ScorePreload = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, scorePose))
                    .setConstantHeadingInterpolation(Math.toRadians(145))
                    .setVelocityConstraint(0.1)
                    .build();

            Grab1 = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose, pickup1front))
                    .setLinearHeadingInterpolation(145,0)
                    .addPath(new BezierLine(pickup1front,pickup1Pose))
                    .setConstantHeadingInterpolation(0)
                    .build();

            Score1 = follower.pathBuilder()
                    .addPath(new BezierLine(pickup1Pose, scorePose))
                    .setLinearHeadingInterpolation(0,Math.toRadians(145))
                    .setVelocityConstraint(0.1)
                    .build();

            Grab2 = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose, pickup2front))
                    .setLinearHeadingInterpolation(145,0)
                    .addPath(new BezierLine(pickup2front,pickup2Pose))
                    .setConstantHeadingInterpolation(0)
                    .build();

            Score2 = follower.pathBuilder()
                    .addPath(new BezierCurve(pickup2Pose,new Pose(60,65), scorePose))
                    .setLinearHeadingInterpolation(0,Math.toRadians(145))
                    .setVelocityConstraint(0.1)
                    .build();
            Park = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose,parkPose))
                    .setConstantHeadingInterpolation(Math.toRadians(145))
                    .build();
        }
    }



    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                shooter.shootNear();
                follower.followPath(paths.ScorePreload,.5,true);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy() && shooter.isShooterAtTargetVelocity()) {
                    intakeWheel.IntakeOn();
                    launcherWheel.LauncherOn();
                    if (pathTimer.getElapsedTimeSeconds() > 4) {
                        intakeWheel.IntakeOff();
                        launcherWheel.LauncherOff();
                        follower.followPath(paths.Grab1,true);
                        intakeWheel.IntakeOn();
                        launcherWheel.LauncherSpit();
                        setPathState(2);
                    }
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Score1, true);
                    if(pathTimer.getElapsedTimeSeconds()>6) {
                        intakeWheel.IntakeOff();
                        launcherWheel.LauncherOff();
                        setPathState(3);
                    }
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    intakeWheel.IntakeOn();
                    launcherWheel.LauncherOn();
                    if (pathTimer.getElapsedTimeSeconds() > 5) {
                        intakeWheel.IntakeOff();
                        launcherWheel.LauncherOff();
                        follower.followPath(paths.Grab2,true);
                        intakeWheel.IntakeOn();
                        launcherWheel.LauncherSpit();
                        setPathState(4);
                    }
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 4.5) {
                        intakeWheel.IntakeOff();
                        launcherWheel.LauncherOff();
                        follower.followPath(paths.Score2, true);
                        setPathState(5);
                    }
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds()>1.5) {
                        intakeWheel.IntakeOn();
                        launcherWheel.LauncherOn();
                        setPathState(6);
                    }
                }
                break;
            case 6:
                if (pathTimer.getElapsedTimeSeconds()>4){
                    follower.followPath(paths.Park,true);
                }
        }
    }
    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}
    