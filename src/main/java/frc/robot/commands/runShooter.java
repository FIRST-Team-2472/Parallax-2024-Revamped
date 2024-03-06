package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystems.*;

public class runShooter extends Command { 

    private ShootingMotorSubsystem shooterSubsystem;
    private IntakeMotorSubsystem intakeSubsystem;
    private Timer timer, timerTwo;



    public runShooter(ShootingMotorSubsystem shooterSubsystem, IntakeMotorSubsystem intakeSubsystem) {
        timer = new Timer();
        timerTwo = new Timer();
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
            shooterSubsystem.runShooterMotors(0.7);

        if (shooterSubsystem.getShooterSpeed() < -3500 || timer.hasElapsed(1)) {
            timerTwo.start();
            shooterSubsystem.runShooterMotors(0.7);
            intakeSubsystem.runPushMotor(1);
            intakeSubsystem.runIntakeMotors(1);
        }
    }

    @Override
    public void end(boolean interrupted) {
       shooterSubsystem.runShooterMotors(0);
       intakeSubsystem.runPushMotor(0);
       intakeSubsystem.runIntakeMotors(0);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(1.5) || timerTwo.hasElapsed(0.3);
    }
}
