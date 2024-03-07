package frc.robot.commands.DefaultCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystems.*;

public class ShooterMotorsCmd extends Command {
    // Suppliers are used so we can get constant updates to the values
    private Double shooterMotorsSpeed;
    private Supplier<Boolean> shooterMotorsSpeaker, shooterMotorsAmp, reversed;
    private ShootingMotorSubsystem shootingMotorSubsystem;;

    public ShooterMotorsCmd(ShootingMotorSubsystem shootingMotorSubsystem, Supplier<Boolean> shooterMotorsSpeaker, Supplier<Boolean> shooterMotorsAmp, Supplier<Boolean> reversed){
        this.shooterMotorsSpeaker = shooterMotorsSpeaker;
        this.shooterMotorsAmp = shooterMotorsAmp;
        this.shootingMotorSubsystem = shootingMotorSubsystem;
        this.reversed = reversed;
        addRequirements(shootingMotorSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();
        //this is a child class which inherits some code so we need to call the constructor
        // of the parent class in the 1st line
    }

    @Override
    public void execute() {
        //runs the shooter motor at 75% speed when we fire in speaker and 50% for the amp
        shooterMotorsSpeed = shooterMotorsSpeaker.get() ? .75 : (shooterMotorsAmp.get() ? 0.5 : 0);
        shooterMotorsSpeed = reversed.get() ? -0.1 : shooterMotorsSpeed;
        shootingMotorSubsystem.runShooterMotors(shooterMotorsSpeed);
        
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
