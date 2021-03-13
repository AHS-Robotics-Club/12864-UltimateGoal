package org.firstinspires.ftc.teamcode.commands.groups;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.Com_PickUp;
import org.firstinspires.ftc.teamcode.commands.Com_PutDown;
import org.firstinspires.ftc.teamcode.commands.RapidFireCommand;
import org.firstinspires.ftc.teamcode.commands.rr.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WobbleSubsystem;

@Config
public class FourRing extends SequentialCommandGroup {

    public static double xBox = 53.0, yBox = -60.0;
    public static double shootPosX = -68.2, shootPosY = 42.3;
    public static double secondWobbleX = -36.0, secondWobbleY = -22.4;
    public static double boxTwoX = 45.0, boxTwoY = 0.0;
    public static double finalX = -8.8, finalY = -1.0;

    private Pose2d startPose = new Pose2d(-63.0, -40.0, Math.toRadians(180.0));

    public FourRing(MecanumDriveSubsystem drive, WobbleSubsystem wobbleSystem, ShooterSubsystem shooter){
        drive.setPoseEstimate(startPose);
        Trajectory traj0 = drive.trajectoryBuilder(startPose)
                .strafeLeft(14.5)
                .build();

        Trajectory traj1 = drive.trajectoryBuilder(traj0.end())
                .back(1.0)
                .splineToConstantHeading(new Vector2d(xBox, yBox), 0.0)
                .build();

        Vector2d shootPose = traj1.end().vec().plus(new Vector2d(shootPosX, shootPosY));

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end(), true)
                .lineToConstantHeading(shootPose)
                .build();

//        Vector2d secondWobble = traj2.end().vec().plus(new Vector2d(secWobblePosX, secWobblePosY));

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .splineToLinearHeading(new Pose2d(-20.0,-24.0, 0.0), Math.toRadians(-90.0))
                .build();

        Trajectory trajAlmost4 = drive.trajectoryBuilder(traj3.end(), 0.0)
                .splineToConstantHeading(new Vector2d(secondWobbleX, secondWobbleY), 0.0)
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(trajAlmost4.end(), 0)
                .splineToSplineHeading(traj3.end().plus(new Pose2d(boxTwoX, boxTwoY, Math.toRadians(-179.0))), Math.toRadians(-90.0))
                .splineToConstantHeading(traj1.end().vec().plus(new Vector2d(finalX, finalY)), 0.0)
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end(), 0)
                .forward(35.0)
                .build();

        addCommands(
                new TrajectoryFollowerCommand(drive, traj0),
                new ParallelDeadlineGroup(
                        new TrajectoryFollowerCommand(drive, traj1),
                        new Com_PutDown(wobbleSystem)
                ),
                new InstantCommand(wobbleSystem::openGrabber, wobbleSystem),
                new WaitCommand(600),
                new Com_PickUp(wobbleSystem).raceWith(new WaitCommand(750)),
                new TrajectoryFollowerCommand(drive, traj2),
                new RapidFireCommand(shooter),
                new ParallelDeadlineGroup(
                        new TrajectoryFollowerCommand(drive, traj3),
                        new Com_PutDown(wobbleSystem)
                ),
                new TrajectoryFollowerCommand(drive, trajAlmost4),
                new InstantCommand(wobbleSystem::closeGrabber, wobbleSystem),
                new WaitCommand(800),
                new TrajectoryFollowerCommand(drive, traj4),
                new InstantCommand(wobbleSystem::openGrabber, wobbleSystem),
                new WaitCommand(700),
                new Com_PickUp(wobbleSystem).raceWith(new WaitCommand(300)),
                new TrajectoryFollowerCommand(drive, traj5)
        );
    }
}