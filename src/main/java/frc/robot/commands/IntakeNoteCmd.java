package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystems.*;

public class IntakeNoteCmd extends Command { 

    private IntakeMotorSubsystem intakeSubsystem;
    private Timer timer;
    private double startingDelay, endingDelay;



    public IntakeNoteCmd(IntakeMotorSubsystem intakeSubsystem, double startingDelay, double endingDelay) {
        timer = new Timer();
        this.intakeSubsystem = intakeSubsystem;
        this.startingDelay = startingDelay;
        this.endingDelay = endingDelay;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        timer.start();
        System.out.println("intake command started");
    }

    @Override
    public void execute() {
        if(timer.hasElapsed(startingDelay)){
            System.out.println("should be running motors");
        intakeSubsystem.runIntakeMotors(0.6);
        intakeSubsystem.runPushMotor(0.6);
        }
    }

    @Override
    public void end(boolean interrupted) {
       intakeSubsystem.runIntakeMotors(0);
       intakeSubsystem.runPushMotor(0);
       System.out.println("intake command ended, time: "+timer.get());
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(endingDelay) || intakeSubsystem.getPhotoElectricSensor() == true;
    }
}
