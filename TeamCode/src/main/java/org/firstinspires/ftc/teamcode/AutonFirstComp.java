package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DriveSystem;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_Drive;

import java.util.concurrent.TimeUnit;

@Autonomous(name="Kanye East")
public class AutonFirstComp extends CommandOpMode {
    private Motor fL, bL, fR, bR;

    private DriveSystem mecDrive;
    private ElapsedTime time;
    public GamepadEx m_driverOp, m_toolOp;

    @Override
    public void initialize() {
        fL = new Motor(hardwareMap, "fL");
        fR = new Motor(hardwareMap, "fR");
        bL = new Motor(hardwareMap, "bL");
        bR = new Motor(hardwareMap, "bR");
        //one of our motors is messed up so it has to be inverted woooooo
        bL.setInverted(true);
        time = new ElapsedTime();
        mecDrive = new DriveSystem(fL, fR, bL, bR);

        m_driverOp = new GamepadEx(gamepad1);
        m_toolOp = new GamepadEx(gamepad2);



        register(mecDrive);
        waitForStart();
        time.reset();
        while(opModeIsActive() && time.seconds() < 1.4) {
            fL.set(0.8);
            fR.set(0.8);
            bL.set(0.8);
            bR.set(0.8);
        }
        fL.set(0);
        fR.set(0);
        bL.set(0);
        bR.set(0);
    }
}


