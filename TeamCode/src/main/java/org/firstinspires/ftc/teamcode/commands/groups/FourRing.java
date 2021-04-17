package org.firstinspires.ftc.teamcode.commands.groups;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
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
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WobbleSubsystem;

import java.util.Arrays;

@Config
public class FourRing extends SequentialCommandGroup {

    public static double traj1X = 50.0, traj1Y = -65.0;
    public static double traj2X = 0.0, traj2Y = -36.0, traj2H = 171.0;

    private Pose2d startPose = new Pose2d(-63.0, -40.0, Math.toRadians(180.0));

    public FourRing(MecanumDriveSubsystem drive, WobbleSubsystem wobbleSystem, ShooterSubsystem shooter,
                    IntakeSubsystem intakeSystem){
        drive.setPoseEstimate(startPose);

        Trajectory traj0 = drive.trajectoryBuilder(startPose)
                .back(1.0)
                .build();

        Trajectory trajHalf = drive.trajectoryBuilder(traj0.end())
                .strafeLeft(16)
                .build();

        Trajectory traj1 = drive.trajectoryBuilder(trajHalf.end())
                .back(2.0)
                .splineToSplineHeading(new Pose2d(traj1X, traj1Y, Math.toRadians(177.0)),0.0)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineTo(new Vector2d(-5.0, -10.0))
                .splineToSplineHeading(new Pose2d(-2.0, -8.0, Math.toRadians(traj2H)), 0.0)
                .build();

        //Shoots 3 and intake starts

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .splineToLinearHeading(new Pose2d(-20.0, -15.0, Math.toRadians(-90.0)),0.0)
                .build();

        Trajectory traj3Half = drive.trajectoryBuilder(traj3.end())
                .splineToConstantHeading(new Vector2d(-20.0, -50.0), 0.0,
                        new MinVelocityConstraint(Arrays.asList(
                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                new MecanumVelocityConstraint(45, DriveConstants.TRACK_WIDTH)
                        )),
                        new ProfileAccelerationConstraint(40.0)
                )
                .build();

        //goes to shooter position
        Trajectory traj4 = drive.trajectoryBuilder(traj3Half.end())
                .lineToLinearHeading(new Pose2d(-12.0, -26, Math.toRadians(180.0)))
                .build();

        //shoots and intake stops

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .splineToLinearHeading(new Pose2d(-24.0, -21.0, Math.toRadians(0.0)), 0.0)
                .build();

        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                .lineTo(new Vector2d(-31.0, -22))
                .build();

        Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
                .splineToSplineHeading(new Pose2d(20.0, -50.0, Math.toRadians(179.0)),0.0,
                        new MinVelocityConstraint(Arrays.asList(
                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                new MecanumVelocityConstraint(55, DriveConstants.TRACK_WIDTH)
                        )),
                        new ProfileAccelerationConstraint(55)
                )
                .splineToConstantHeading(new Vector2d(42.5, -64.0), 0.0,
                        new MinVelocityConstraint(Arrays.asList(
                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                new MecanumVelocityConstraint(55, DriveConstants.TRACK_WIDTH)
                        )),
                        new ProfileAccelerationConstraint(55)
                )
                .build();

        Trajectory traj8 = drive.trajectoryBuilder(traj7.end())
                .forward(30.0,new MinVelocityConstraint(Arrays.asList(
                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                        new MecanumVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH)
                        )),
                        new ProfileAccelerationConstraint(55.0)
                )
                .build();

        addCommands(
                new TrajectoryFollowerCommand(drive, traj0),
                new ParallelDeadlineGroup(
                        new TrajectoryFollowerCommand(drive, trajHalf),
                        new Com_PutDown(wobbleSystem)
                ),
                new TrajectoryFollowerCommand(drive, traj1),
                new InstantCommand(wobbleSystem::openGrabber, wobbleSystem),
                new WaitCommand(300),
                new ParallelDeadlineGroup(
                        new TrajectoryFollowerCommand(drive, traj2),
                        new Com_PickUp(wobbleSystem)
                ),
                new RapidFireCommand(shooter),
                new InstantCommand(intakeSystem::autoIntake),
                new TrajectoryFollowerCommand(drive, traj3),
                new TrajectoryFollowerCommand(drive, traj3Half),
                new TrajectoryFollowerCommand(drive, traj4),
                new RapidFireCommand(shooter),
                new InstantCommand(intakeSystem::stop),
                new ParallelDeadlineGroup(
                        new TrajectoryFollowerCommand(drive, traj5),
                        new Com_PutDown(wobbleSystem)
                ),
                new TrajectoryFollowerCommand(drive, traj6),
                new InstantCommand(wobbleSystem::closeGrabber, wobbleSystem),
                new WaitCommand(450),
                new TrajectoryFollowerCommand(drive, traj7),
                new InstantCommand(wobbleSystem::openGrabber, wobbleSystem),
                new WaitCommand(300),
                new ParallelDeadlineGroup(
                        new TrajectoryFollowerCommand(drive, traj8),
                        new Com_EndAutoPickUp(wobbleSystem)
                )
        );
    }
}