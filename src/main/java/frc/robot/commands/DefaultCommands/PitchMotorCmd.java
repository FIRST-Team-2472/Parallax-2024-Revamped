package frc.robot.commands.DefaultCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.ArmSubsystems.*;

public class PitchMotorCmd extends Command {
    // Suppliers are used so we can get constant updates to the values
    private Supplier<Double> pitchMotor;
    private Double pitchMotorSpeed;
    private Supplier<Boolean> intakebutton;
    private PitchMotorSubsystem armPitchSubsystem;
    private Timer timer = new Timer();

    public PitchMotorCmd(PitchMotorSubsystem armPitchSubsystem, Supplier<Double> pitchMotor, Supplier<Boolean> intakebutton){
        this.pitchMotor = pitchMotor;
        this.armPitchSubsystem = armPitchSubsystem;
        this.intakebutton = intakebutton;
        addRequirements(armPitchSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();
        //this is a child class which inherits some code so we need to call the constructor
        // of the parent class in the 1st line
        timer.start();
    }

    @Override
    public void execute() {
        //constantly update the pitch motor input
        pitchMotorSpeed = pitchMotor.get();

        // applies a deadband
        if (pitchMotorSpeed < OIConstants.kArmDeadband && pitchMotorSpeed > -OIConstants.kArmDeadband) 
            pitchMotorSpeed = 0.0;
        pitchMotorSpeed *= 0.45;//slows down the arm
        if(!intakebutton.get())
            armPitchSubsystem.runPitchMotor(pitchMotorSpeed);
        else
            armPitchSubsystem.runPitchMotor(pitchMotorSpeed, true);
        
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
