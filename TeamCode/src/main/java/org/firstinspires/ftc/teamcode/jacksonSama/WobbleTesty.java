package org.firstinspires.ftc.teamcode.jacksonSama;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Aiden")
public class WobbleTesty extends LinearOpMode {

    Motor m_motor;

    @Override
    public void runOpMode() throws InterruptedException {
        m_motor = new Motor(hardwareMap, "wobble", Motor.GoBILDA.BARE);
// set the run mode
        m_motor.setRunMode(Motor.RunMode.PositionControl);
        m_motor.setPositionCoefficient(0.01);
        m_motor.setPositionTolerance(10);   // allowed maximum error
        m_motor.setTargetPosition(10000);
        m_motor.set(0);
        m_motor.setDistancePerPulse(Math.PI * 2.0 / m_motor.getCPR());

        waitForStart();

// perform the control loop
        while (opModeIsActive() && !m_motor.atTargetPosition()) {
            m_motor.set(0.5);
            telemetry.addData("current positon", m_motor.getCurrentPosition());
            telemetry.addData("at target", m_motor.atTargetPosition());
            telemetry.update();
        }
        m_motor.stopMotor(); // stop the motor

    }
}
