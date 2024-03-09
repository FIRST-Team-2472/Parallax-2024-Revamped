package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystems.*;

public class runShooter extends Command { 

    private ShootingMotorSubsystem shooterSubsystem;
    private IntakeMotorSubsystem intakeSubsystem;
    private Timer overideTimer, timerTwo;
    private double speed;


    public runShooter(ShootingMotorSubsystem shooterSubsystem, IntakeMotorSubsystem intakeSubsystem, double speed) {
        overideTimer = new Timer();
        timerTwo = new Timer();
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.speed = speed;
        addRequirements(shooterSubsystem);
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        overideTimer.stop();
        overideTimer.reset();
        overideTimer.start();

        timerTwo.stop();
        timerTwo.reset();
        System.out.println("running");
        System.out.println(""+overideTimer.get());
    }

    @Override
    public void execute() {
            shooterSubsystem.runShooterMotors(speed);

        if (shooterSubsystem.getShooterSpeed() < -3500 || overideTimer.hasElapsed(1) || speed < 0.6) {
            if(timerTwo.get() != 0.0)
                timerTwo.start();
            intakeSubsystem.runPushMotor(1);
            intakeSubsystem.runIntakeMotors(1);
        }
    }

    @Override
    public void end(boolean interrupted) {
       shooterSubsystem.runShooterMotors(0);
       intakeSubsystem.runPushMotor(0);
       intakeSubsystem.runIntakeMotors(0);
       System.out.println("interrupted");
    }

    @Override
    public boolean isFinished() {

        return overideTimer.hasElapsed(1.5) || timerTwo.hasElapsed(0.3);
    }
}
