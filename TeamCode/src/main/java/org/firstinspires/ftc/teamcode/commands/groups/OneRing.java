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

import org.firstinspires.ftc.teamcode.commands.Com_EndAutoPickUp;
import org.firstinspires.ftc.teamcode.commands.Com_Intake;
import org.firstinspires.ftc.teamcode.commands.Com_PickUp;
import org.firstinspires.ftc.teamcode.commands.Com_PutDown;
import org.firstinspires.ftc.teamcode.commands.RapidFireCommand;
import org.firstinspires.ftc.teamcode.commands.rr.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.rr.TurnCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WobbleSubsystem;

import java.time.Instant;

@Config
public class OneRing extends SequentialCommandGroup {

    public static double xBox = 26.0, yBox = -44.0;
    public static double shootPosX = -38.0, shootPosY = 24.0;
    public static double secondWobbleX = -32, secondWobbleY = -22.0;
    public static double finalX = -5.5, finalY = 1.0;

    private Pose2d startPose = new Pose2d(-63.0, -40.0, Math.toRadians(180.0));

    public OneRing(MecanumDriveSubsystem drive, WobbleSubsystem wobbleSystem, ShooterSubsystem shooter,
                   IntakeSubsystem intakeSystem){
        drive.setPoseEstimate(startPose);

        Trajectory traj0 = drive.trajectoryBuilder(startPose)
                .strafeLeft(14.5)
                .build();

        Trajectory traj1 = drive.trajectoryBuilder(traj0.end())
                .back(1.0)
                .splineToConstantHeading(new Vector2d(1.0, -60.0), 0.0)
                .splineToConstantHeading(new Vector2d(xBox, yBox), 0.0)
                .build();

//                        new MinVelocityConstraint(Arrays.asList(
//                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                new MecanumVelocityConstraint(50, DriveConstants.TRACK_WIDTH)
//                        )),
//                                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
//                        )

        Vector2d shootPose = traj1.end().vec().plus(new Vector2d(shootPosX, shootPosY));

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end(), true)
                .lineToConstantHeading(shootPose)
                .build();

//        Vector2d secondWobble = traj2.end().vec().plus(new Vector2d(secWobblePosX, secWobblePosY));

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end(), Math.toRadians(192.0))
                .splineToLinearHeading(new Pose2d(-20.0,-20.0, 0.0), Math.toRadians(-90.0))
                .build();

        Trajectory trajAlmost4 = drive.trajectoryBuilder(traj3.end())
                .splineToConstantHeading(new Vector2d(secondWobbleX, secondWobbleY), 0.0)
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(trajAlmost4.end())
                .lineToConstantHeading(traj3.end().vec().plus(new Vector2d(0, 8)))
                .splineToSplineHeading((new Pose2d(0.0, -18, Math.toRadians(181.0))), Math.toRadians(-90.0))
                .splineToConstantHeading(traj1.end().vec().plus(new Vector2d(finalX, finalY)), 0.0)
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .splineToConstantHeading(traj4.end().vec().plus(new Vector2d(-35.0, 7.0)), 0.0)
                .build();

        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                .splineToSplineHeading(traj4.end().plus(new Pose2d(-10.0, 1.0, 0.0)), 0.0)
                .build();

        addCommands(
                new ParallelDeadlineGroup(
                        new TrajectoryFollowerCommand(drive, traj0),
                        new Com_PutDown(wobbleSystem)
                ),
                new TrajectoryFollowerCommand(drive, traj1),
                new InstantCommand(wobbleSystem::openGrabber, wobbleSystem),
                new WaitCommand(700),
                new Com_PickUp(wobbleSystem).raceWith(new WaitCommand(750)),
                new TurnCommand(drive, Math.toRadians(12)),
                new TrajectoryFollowerCommand(drive, traj2),
                new RapidFireCommand(shooter),
                new ParallelDeadlineGroup(
                        new TrajectoryFollowerCommand(drive, traj3),
                        new Com_PutDown(wobbleSystem)
                ),
                new TrajectoryFollowerCommand(drive, trajAlmost4),
                new InstantCommand(wobbleSystem::closeGrabber, wobbleSystem),
                new WaitCommand(700),
                new ParallelDeadlineGroup(
                        new TrajectoryFollowerCommand(drive, traj4),
                        new Com_PickUp(wobbleSystem)
                ),
                new Com_PutDown(wobbleSystem).raceWith(new WaitCommand(400)),
                new InstantCommand(wobbleSystem::openGrabber, wobbleSystem),
                new WaitCommand(500),
                new ParallelDeadlineGroup(
                        new TrajectoryFollowerCommand(drive, traj5),
                        new Com_EndAutoPickUp(wobbleSystem),
                        new InstantCommand(intakeSystem::start)
                ),
                new WaitCommand(400).andThen(new InstantCommand(intakeSystem::stop)),
                new TurnCommand(drive, Math.toRadians(10.0)),
                new RapidFireCommand(shooter, 1),
                new TrajectoryFollowerCommand(drive, traj6)
        );
    }
}
