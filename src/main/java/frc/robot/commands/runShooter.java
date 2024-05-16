package frc.robot.commands;


import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystems.*;

public class runShooter extends Command { 

    private ShootingMotorSubsystem shooterSubsystem;
    private IntakeMotorSubsystem intakeSubsystem;
    private Timer overideTimer, timerTwo;
    private Supplier<Double> supplierSpeed;
    private int rpm;
public runShooter(ShootingMotorSubsystem shooterSubsystem, IntakeMotorSubsystem intakeSubsystem, double supplierSpeed) {
        overideTimer = new Timer();
        timerTwo = new Timer();
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        //(leftJoystick.getThrottle()*-1 + 1) * 0.2 + 0.1
        addRequirements(shooterSubsystem);
        addRequirements(intakeSubsystem);
        rpm = 3500;
    }
    public runShooter(ShootingMotorSubsystem shooterSubsystem, IntakeMotorSubsystem intakeSubsystem, double supplierSpeed, int rpm) {
        overideTimer = new Timer();
        timerTwo = new Timer();
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        //(leftJoystick.getThrottle()*-1 + 1) * 0.2 + 0.1
        addRequirements(shooterSubsystem);
        addRequirements(intakeSubsystem);
        rpm = 3500;
    }
    public runShooter(ShootingMotorSubsystem shooterSubsystem, IntakeMotorSubsystem intakeSubsystem, Supplier<Double> supplierSpeed) {
        overideTimer = new Timer();
        timerTwo = new Timer();
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.supplierSpeed = supplierSpeed;
        //(leftJoystick.getThrottle()*-1 + 1) * 0.2 + 0.1
        addRequirements(shooterSubsystem);
        addRequirements(intakeSubsystem);
        rpm = 3500;
    }
  
    @Override
    public void initialize() {
        overideTimer.stop();
        overideTimer.reset();
        overideTimer.start();

        timerTwo.stop();
        timerTwo.reset();
        System.out.println("running");
        System.out.println("override timer: "+overideTimer.get());
        System.out.println(""+ -1*rpm);
    }

    @Override
    public void execute() {
        double tempSpeed = supplierSpeed.get();
        tempSpeed = (tempSpeed*-1 + 1) * 0.2 + 0.3;
            shooterSubsystem.runShooterMotors(tempSpeed);

        if (shooterSubsystem.getShooterSpeed() < -rpm || overideTimer.hasElapsed(2)) {
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

        return overideTimer.hasElapsed(2.5) || timerTwo.hasElapsed(0.3);
    }
}
