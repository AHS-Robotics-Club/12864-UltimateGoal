package org.firstinspires.ftc.teamcode.commands.groups;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.apache.commons.math3.stat.descriptive.moment.VectorialCovariance;
import org.firstinspires.ftc.teamcode.commands.Com_EndAutoPickUp;
import org.firstinspires.ftc.teamcode.commands.Com_PickUp;
import org.firstinspires.ftc.teamcode.commands.Com_PutDown;
import org.firstinspires.ftc.teamcode.commands.RapidFireCommand;
import org.firstinspires.ftc.teamcode.commands.rr.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.rr.TurnCommand;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WobbleSubsystem;

import java.util.Vector;

@Config
public class ZeroRing extends SequentialCommandGroup{

    public static double secondWobbleX = -33.3, secondWobbleY = -20.5;

    private Pose2d startPose = new Pose2d(-63.0, -40.0, Math.toRadians(180.0));

    public ZeroRing(MecanumDriveSubsystem drive, WobbleSubsystem wobbleSystem, ShooterSubsystem shooter){
        drive.setPoseEstimate(startPose);
        Trajectory traj0 = drive.trajectoryBuilder(startPose)
                .strafeLeft(14.5)
                .build();

        Trajectory traj1 = drive.trajectoryBuilder(traj0.end())
                .back(1.0)
                .splineToConstantHeading(new Vector2d(2.0, -60.0), 0.0)
                .build();

        Vector2d shootPose = traj1.end().vec().plus(new Vector2d(-15.0, 25.3));

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end(), true)
                .lineToConstantHeading(shootPose)
                .build();

        //-34 , -24
//        Vector2d secondWobble = traj2.end().vec().plus(new Vector2d(-14.0, 15.5));

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end(), 0.0)
                .splineToLinearHeading(new Pose2d(-20.0, -20.0, 0.0), Math.toRadians(-90.0))
                .build();

        Trajectory trajAlmost4 = drive.trajectoryBuilder(traj3.end())
                .splineToConstantHeading(new Vector2d(secondWobbleX, secondWobbleY), 0.0)
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(trajAlmost4.end(), 0.0)
                .splineToSplineHeading(trajAlmost4.end().plus(new Pose2d(16.0, 0.0, Math.toRadians(180.0))), 0.0)
                .splineToConstantHeading(traj1.end().vec().plus(new Vector2d(-9.0, -4.0)), 0.0)
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .splineToConstantHeading(traj4.end().vec().plus(new Vector2d(-10.0, 16.0)), 0.0)
//                .splineTo(traj4.end().vec().plus(new Vector2d(-16.0, 16.0)), 0.0)
                .splineTo(traj4.end().vec().plus(new Vector2d(25.0, 24.0)), 0.0)
                .build();

        addCommands(
                new ParallelDeadlineGroup(
                        new TrajectoryFollowerCommand(drive, traj0),
                        new Com_PutDown(wobbleSystem)
                ),
                new TrajectoryFollowerCommand(drive, traj1),
                new InstantCommand(wobbleSystem::openGrabber, wobbleSystem),
                new WaitCommand(500),
                new Com_PickUp(wobbleSystem).raceWith(new WaitCommand(750)),
                new TrajectoryFollowerCommand(drive, traj2),
                new TurnCommand(drive, Math.toRadians(15)),
                new RapidFireCommand(shooter),
                new ParallelDeadlineGroup(
                    new TrajectoryFollowerCommand(drive, traj3),
                    new Com_PutDown(wobbleSystem)
                ),
                new TrajectoryFollowerCommand(drive, trajAlmost4),
                new InstantCommand(wobbleSystem::closeGrabber, wobbleSystem),
                new WaitCommand(1000),
                new TrajectoryFollowerCommand(drive, traj4),
                new InstantCommand(wobbleSystem::openGrabber, wobbleSystem),
                new WaitCommand(600),
                new ParallelCommandGroup(
                        new Com_EndAutoPickUp(wobbleSystem),
                        new TrajectoryFollowerCommand(drive, traj5)
                )
        );
    }
}
