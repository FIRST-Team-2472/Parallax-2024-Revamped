package frc.robot.commands.DefaultCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.ArmSubsystems.*;

public class IntakeMotorCmd extends Command {
    // Suppliers are used so we can get constant updates to the values
    private Double intakeMotorsSpeed, pushMotorSpeed;
    private Supplier<Boolean> intakeMotorsRunning,  reversed;
    private boolean sensed;
    private IntakeMotorSubsystem intakeMotorSubsystem;
    private Timer timer = new Timer();

    public IntakeMotorCmd(IntakeMotorSubsystem intakeMotorSubsystem, 
         Supplier<Boolean> intakeMotorsRunning, Supplier<Boolean> reversed){
        sensed = false;
        this.intakeMotorsRunning = intakeMotorsRunning;
        this.intakeMotorSubsystem = intakeMotorSubsystem;
        this.reversed = reversed;
        addRequirements(intakeMotorSubsystem);
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
    
        if(intakeMotorSubsystem.getPhotoElectricSensor()){
            sensed = true;
            timer.reset();
        }
        
        //runs the push motor when ready to fire or during intaking, until it hit the sensor
        pushMotorSpeed = 0.0;
        pushMotorSpeed = reversed.get() ? -.2 : pushMotorSpeed;
        pushMotorSpeed = intakeMotorsRunning.get() ? 0.6 : pushMotorSpeed;
        intakeMotorSubsystem.runPushMotor(pushMotorSpeed);


        //runs the intake motors until the sensor is triggered
        intakeMotorsSpeed = 0.0;
        intakeMotorsSpeed = reversed.get() ? -0.2 : intakeMotorsSpeed;
        intakeMotorsSpeed = intakeMotorsRunning.get() ? 0.6 : intakeMotorsSpeed;
        intakeMotorSubsystem.runIntakeMotors(intakeMotorsSpeed);
        
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
