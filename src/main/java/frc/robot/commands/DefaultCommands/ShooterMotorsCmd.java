package frc.robot.commands.DefaultCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.ArmSubsystems.*;

public class ShooterMotorsCmd extends Command {
    // Suppliers are used so we can get constant updates to the values
    private Supplier<Double> pitchMotor;
    private Double intakeMotorsSpeed, shooterMotorsSpeed, pushMotorSpeed, pitchMotorSpeed;
    private Supplier<Boolean> intakeMotorsRunning, shooterMotorsSpeaker, shooterMotorsAmp, reversed;
    private boolean sensed;
    private PitchMotorSubsystem armPitchSubsystem;
    private ShootingMotorSubsystem shootingMotorSubsystem;
    private IntakeMotorSubsystem intakeMotorSubsystem;
    private Timer timer = new Timer();

    public ShooterMotorsCmd(PitchMotorSubsystem armPitchSubsystem, ShootingMotorSubsystem shootingMotorSubsystem, IntakeMotorSubsystem intakeMotorSubsystem, Supplier<Double> pitchMotor, Supplier<Boolean> shooterMotorsSpeaker, Supplier<Boolean> shooterMotorsAmp, 
         Supplier<Boolean> intakeMotorsRunning, Supplier<Boolean> reversed){
        this.pitchMotor = pitchMotor;
        sensed = false;
        this.shooterMotorsSpeaker = shooterMotorsSpeaker;
        this.shooterMotorsAmp = shooterMotorsAmp;
        this.intakeMotorsRunning = intakeMotorsRunning;
        this.shootingMotorSubsystem = shootingMotorSubsystem;
        this.intakeMotorSubsystem = intakeMotorSubsystem;
        this.reversed = reversed;
        addRequirements(armPitchSubsystem);
        addRequirements(shootingMotorSubsystem);
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
        //runs the shooter motor at 75% speed when we fire in speaker and 50% for the amp
        shooterMotorsSpeed = shooterMotorsSpeaker.get() ? .75 : (shooterMotorsAmp.get() ? 0.5 : 0);
        shooterMotorsSpeed = reversed.get() ? -0.1 : shooterMotorsSpeed;
        shootingMotorSubsystem.runShooterMotors(shooterMotorsSpeed);


        if(shootingMotorSubsystem.getShooterSpeed() < -3500 || timer.hasElapsed(2)){
            sensed = false;
        }
        

        //runs the push motor when ready to fire or during intaking, until it hit the sensor
        pushMotorSpeed = shootingMotorSubsystem.getShooterSpeed() < -3500 ? 0.5 : (intakeMotorsRunning.get() && !sensed ? 0.4 : shooterMotorsAmp.get() ? .6 : 0);
        pushMotorSpeed = reversed.get() ? -.2 : pushMotorSpeed;
        intakeMotorSubsystem.runPushMotor(pushMotorSpeed);


        //runs the intake motors until the sensor is triggered
        
        intakeMotorsSpeed = ((intakeMotorsRunning.get() && !sensed)|| shootingMotorSubsystem.getShooterSpeed() < -3500|| shooterMotorsAmp.get()) ? 0.4 : 0;
        
        SmartDashboard.putNumber("Shooter speed", shootingMotorSubsystem.getShooterSpeed());
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
