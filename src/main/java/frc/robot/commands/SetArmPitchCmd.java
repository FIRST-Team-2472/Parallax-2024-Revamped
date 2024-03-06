package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmMotorsConstants;
import frc.robot.subsystems.ArmSubsystems.PitchMotorSubsystem;

public class SetArmPitchCmd extends Command {
    private PitchMotorSubsystem pitchMotorSubsystem;
    private double angleDeg;
    private final Timer timer;
    
    public SetArmPitchCmd(PitchMotorSubsystem pitchMotorSubsystem, double angleDeg) {
        this.angleDeg = angleDeg;
        this.timer = new Timer();
        this.pitchMotorSubsystem = pitchMotorSubsystem;
        addRequirements(pitchMotorSubsystem);
    }
    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        pitchMotorSubsystem.runPitchMotorWithKP(angleDeg);
    } 

    @Override
    public void end(boolean interrupted) {
        pitchMotorSubsystem.runPitchMotor(0.0);
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(pitchMotorSubsystem.getEncoderDeg() - angleDeg) < 0.5) || (timer.hasElapsed(3));
    }

}
