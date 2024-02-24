package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmMotorsSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class OverrideCmd extends Command{

    private SwerveSubsystem swerveSubsystem;
    private ArmMotorsSubsystem armSubsystem;
    
    public OverrideCmd(SwerveSubsystem swerveSubsystem, ArmMotorsSubsystem armSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
        addRequirements(swerveSubsystem); 
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }

}
