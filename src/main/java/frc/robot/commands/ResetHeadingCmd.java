package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;


public class ResetHeadingCmd extends Command{

    SwerveSubsystem swerveSubsystem;

    
    /**
     *  a toggle button that changes some booleans which cause
     * the arm, and the robot to angle to the speaker, rev up the shooter motors and instantly ends
     *  warning, does cancel any current commands with those subsystems
     * @param swerveSubsystem the swerve subsystem
     * @param pitchMotorSubsystem the pitch motor subsytem
     * @param shootingMotorSubsystem the shooting motor subsystem
     */

    public ResetHeadingCmd(SwerveSubsystem swerveSubsystem){
        this.swerveSubsystem = swerveSubsystem;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize(){
        swerveSubsystem.zeroHeading();
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
