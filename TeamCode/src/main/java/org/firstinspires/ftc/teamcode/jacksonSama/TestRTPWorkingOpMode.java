package org.firstinspires.ftc.teamcode.jacksonSama;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@Autonomous(name="RTP Working")
public class TestRTPWorkingOpMode extends LinearOpMode {

    Motor m_motor;

    @Override
    public void runOpMode() throws InterruptedException {
        m_motor = new Motor(hardwareMap, "test", Motor.GoBILDA.RPM_312);
// set the run mode
        m_motor.setRunMode(Motor.RunMode.PositionControl);

        m_motor.setPositionCoefficient(0.02);
        m_motor.setTargetPosition(1200);      // an integer representing

// set the target position
// set the tolerance
        m_motor.setPositionTolerance(10);   // allowed maximum error

        m_motor.set(0);

        waitForStart();

// perform the control loop
        while (opModeIsActive() && !m_motor.atTargetPosition()) {
            m_motor.set(0.5);
            telemetry.addData("current positon", m_motor.getDistance());
            telemetry.addData("at target", m_motor.atTargetPosition());
            telemetry.update();
        }
        m_motor.stopMotor(); // stop the motor

    }

}