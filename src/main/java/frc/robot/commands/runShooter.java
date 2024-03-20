package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystems.*;

public class runShooter extends Command { 

    private ShootingMotorSubsystem shooterSubsystem;
    private IntakeMotorSubsystem intakeSubsystem;
    private Timer timer, timerTwo;
    private double speed;


    public runShooter(ShootingMotorSubsystem shooterSubsystem, IntakeMotorSubsystem intakeSubsystem, double speed) {
        timer = new Timer();
        timerTwo = new Timer();
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.speed = speed;
        addRequirements(shooterSubsystem);
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        timer.stop();
        timer.reset();
        timer.start();

        timerTwo.stop();
        timerTwo.reset();
        System.out.println("running");
        System.out.println(""+timer.get());
    }

    @Override
    public void execute() {
            shooterSubsystem.runShooterMotors(speed);

        if (shooterSubsystem.getShooterSpeed() < -3500 || timer.hasElapsed(1) || speed < 0.6) {
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
       if (interrupted) System.out.println("interrupted");
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(1.5) || timerTwo.hasElapsed(0.3);
    }
}
