package org.firstinspires.ftc.teamcode.jacksonSama;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@Disabled
@Autonomous(name="PursuitingPurely")

public class PurePursuitTesting extends CommandOpMode {

    static final double TRACKWIDTH = 13.7;
    static final double WHEEL_DIAMETER = 4.0;    // inches
    static double TICKS_TO_INCHES;
    static final double CENTER_WHEEL_OFFSET = 2.4;

    private Motor fL, bL, fR, bR;
    private MecanumDrive mecDrive;
    private PurePursuitCommand bigppCom;
    private HolonomicOdometry m_robotOdometry;
    private OdometrySubsystem m_odometry;
    private MotorEx leftEncoder, rightEncoder, centerEncoder;

    @Override
    public void initialize() {
        fL = new Motor(hardwareMap, "fL");
        fR = new Motor(hardwareMap, "fR");
        bL = new Motor(hardwareMap, "bL");
        bR = new Motor(hardwareMap, "bR");

        mecDrive = new MecanumDrive(fL, fR, bL, bR);
        leftEncoder = new MotorEx(hardwareMap, "leftEncoder");
        rightEncoder = new MotorEx(hardwareMap, "rightEncoder");
        centerEncoder = new MotorEx(hardwareMap, "centerEncoder");

        // calculate multiplier
        TICKS_TO_INCHES = WHEEL_DIAMETER * Math.PI / leftEncoder.getCPR();

        // create our odometry object and subsystem
        m_robotOdometry = new HolonomicOdometry(
                () -> leftEncoder.getCurrentPosition() * TICKS_TO_INCHES,
                () -> rightEncoder.getCurrentPosition() * TICKS_TO_INCHES,
                () -> centerEncoder.getCurrentPosition() * TICKS_TO_INCHES,
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );
        m_odometry = new OdometrySubsystem(m_robotOdometry);

        bigppCom = new PurePursuitCommand(mecDrive, m_odometry, new StartWaypoint(0,0),
                new GeneralWaypoint(200,0,0.8,0.8,30),
                new EndWaypoint(400, 0, 0, 0.5,
                        0.5, 30, 0.8, 1));

    }
}
