package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm_Motors_Subsystem;

public class runShooter extends Command { 

    private Arm_Motors_Subsystem armSubsystem;
    private Timer timer;



    public runShooter(Arm_Motors_Subsystem armSubsystem) {
        timer = new Timer();
        this.armSubsystem = armSubsystem;
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        armSubsystem.runShooterMotors(0.5);
        if (timer.hasElapsed(1.5)) {
            armSubsystem.runPushMotor(0.5);
        }
    }

    @Override
    public void end(boolean interrupted) {
       armSubsystem.runShooterMotors(0);
       armSubsystem.runPushMotor(0);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(0.5);
    }
}
