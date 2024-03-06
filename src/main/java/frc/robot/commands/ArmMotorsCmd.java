package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmMotorsConstants;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmMotorsConstants.ShooterMotors;
import frc.robot.subsystems.ArmSubsystems.*;
import frc.robot.Constants.SensorConstants;

public class ArmMotorsCmd extends Command {
    // Suppliers are used so we can get constant updates to the values
    private Supplier<Double> pitchMotor;
    private SetArmPitchCmd pitchCmd;
    private Double intakeMotorsSpeed, shooterMotorsSpeed, pushMotorSpeed, pitchMotorSpeed;
    private Supplier<Boolean> intakeMotorsRunning, shooterMotorsSpeaker, shooterMotorsAmp, reversed;
    private boolean fired, sensed;
    private PitchMotorSubsystem armPitchSubsystem;
    private ShootingMotorSubsystem shootingMotorSubsystem;
    private IntakeMotorSubsystem intakeMotorSubsystem;
    private Timer timer = new Timer();

    public ArmMotorsCmd(PitchMotorSubsystem armPitchSubsystem, ShootingMotorSubsystem shootingMotorSubsystem, IntakeMotorSubsystem intakeMotorSubsystem, Supplier<Double> pitchMotor, Supplier<Boolean> shooterMotorsSpeaker, Supplier<Boolean> shooterMotorsAmp, 
         Supplier<Boolean> intakeMotorsRunning, Supplier<Boolean> reversed){
        this.pitchMotor = pitchMotor;
        fired = false;
        sensed = false;
        this.shooterMotorsSpeaker = shooterMotorsSpeaker;
        this.shooterMotorsAmp = shooterMotorsAmp;
        this.intakeMotorsRunning = intakeMotorsRunning;
        this.armPitchSubsystem = armPitchSubsystem;
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
        //constantly update the pitch motor input
        pitchMotorSpeed = pitchMotor.get();

        // applies a deadband
        if (pitchMotorSpeed < OIConstants.kArmDeadband && pitchMotorSpeed > -OIConstants.kArmDeadband) 
            pitchMotorSpeed = 0.0;
        pitchMotorSpeed *= 0.45;//slows down the arm
        if(!intakeMotorsRunning.get())
            armPitchSubsystem.runPitchMotor(pitchMotorSpeed);
        else
            armPitchSubsystem.runPitchMotor(pitchMotorSpeed, true);

        //runs the shooter motor at 75% speed when we fire in speaker and 50% for the amp
        shooterMotorsSpeed = shooterMotorsSpeaker.get() ? .75 : (shooterMotorsAmp.get() ? 0.5 : 0);
        shooterMotorsSpeed = reversed.get() ? -0.1 : shooterMotorsSpeed;
        shootingMotorSubsystem.runShooterMotors(shooterMotorsSpeed);

        if(intakeMotorSubsystem.getPhotoElectricSensor()){
            sensed = true;
            timer.reset();
        }
        if(shootingMotorSubsystem.getShooterSpeed() < -3500 || timer.hasElapsed(2)){
            sensed = false;
        }
        

        //runs the push motor when ready to fire or during intaking, until it hit the sensor
        pushMotorSpeed = shootingMotorSubsystem.getShooterSpeed() < -3500 ? 0.5 : (intakeMotorsRunning.get() && !sensed ? 0.4 : shooterMotorsAmp.get() ? .6 : 0);
        pushMotorSpeed = reversed.get() ? -.2 : pushMotorSpeed;
        intakeMotorSubsystem.runPushMotor(pushMotorSpeed);


        //runs the intake motors until the sensor is triggered
        
        intakeMotorsSpeed = ((intakeMotorsRunning.get() && !sensed)|| shootingMotorSubsystem.getShooterSpeed() < -3500|| shooterMotorsAmp.get()) ? 0.4 : 0;
        intakeMotorsSpeed = reversed.get() ? -0.2 : intakeMotorsSpeed;
        intakeMotorSubsystem.runIntakeMotors(intakeMotorsSpeed);
        
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
