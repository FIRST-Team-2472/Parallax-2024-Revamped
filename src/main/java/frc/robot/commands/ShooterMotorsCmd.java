package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterMotorsCmd extends Command{
    private Supplier<Double> topMotorSpeed;
    private Supplier<Double> bottomMotorSpeed;
    private Shooter shooter;
    public ShooterMotorsCmd(Shooter shooter, Supplier<Double> topMotorSpeed, Supplier<Double> bottomMotorSpeed){
        this.topMotorSpeed = topMotorSpeed;
        this.bottomMotorSpeed = bottomMotorSpeed;
        this.shooter = shooter;
    }
    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        shooter.runshootermotors(topMotorSpeed.get(),bottomMotorSpeed.get());
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
