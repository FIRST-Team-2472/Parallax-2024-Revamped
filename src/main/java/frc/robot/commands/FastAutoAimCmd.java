package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AimPoint;
import frc.robot.AutoAiming;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ArmSubsystems.IntakeMotorSubsystem;
import frc.robot.subsystems.ArmSubsystems.PitchMotorSubsystem;
import frc.robot.subsystems.ArmSubsystems.ShootingMotorSubsystem;

public class FastAutoAimCmd extends Command{
//rename this class if it is better than normal autoshooting

    SwerveSubsystem swerveSubsystem;
    PitchMotorSubsystem pitchMotorSubsystem;
    ShootingMotorSubsystem shootingMotorSubsystem;
    IntakeMotorSubsystem intakeMotorSubsystem;
    Timer overideTimer, timerTwo;
    AimPoint aimPoint;
    
    /**
     * A fully contained auto-aim command that revs the shooter motors on the way and used a special pid for the pitch motor
     * 
     * @param swerveSubsystem        the swerve subsytstem
     * @param pitchMotorSubsystem    the pitch motor subsystem
     * @param shootingMotorSubsystem the shooting motor subsystem
     * @param intakeMotorSubsystem   the intake motor subsystem
     */
    public FastAutoAimCmd(SwerveSubsystem swerveSubsystem, PitchMotorSubsystem pitchMotorSubsystem, 
    ShootingMotorSubsystem shootingMotorSubsystem, IntakeMotorSubsystem intakeMotorSubsystem){
        this.swerveSubsystem = swerveSubsystem;
        this.intakeMotorSubsystem = intakeMotorSubsystem;
        this.pitchMotorSubsystem = pitchMotorSubsystem;
        this.shootingMotorSubsystem = shootingMotorSubsystem;
    }

    @Override
    public void initialize() {
        overideTimer.restart();

        timerTwo.stop();
        timerTwo.reset();

        System.out.println("faster auto-aim command started");

        aimPoint = AutoAiming.getAimPoint(swerveSubsystem.getPose());
    }

    @Override
    public void execute() {
        pitchMotorSubsystem.runPitchMotorWithFasterKP(aimPoint.getPitchAngle());
        swerveSubsystem.executeRotateToAngle(Rotation2d.fromDegrees(aimPoint.getYawAngle()));

        // depending on power consumption the shooter motor may need to be held back to start
        shootingMotorSubsystem.runShooterMotors(0.9);

        if (shootingMotorSubsystem.getShooterSpeed() < -3500 && (Math.abs(pitchMotorSubsystem.getEncoderDeg() - aimPoint.getPitchAngle()) < 0.5) 
        && swerveSubsystem.isAtAngle(Rotation2d.fromDegrees(aimPoint.getYawAngle()))
        || overideTimer.hasElapsed(2)) {
            if(timerTwo.get() != 0.0)
                timerTwo.start();
            intakeMotorSubsystem.runPushMotor(1);
            intakeMotorSubsystem.runIntakeMotors(1);
        }
    }

    @Override
    public void end(boolean interrupted) {
       shootingMotorSubsystem.runShooterMotors(0);
       intakeMotorSubsystem.runPushMotor(0);
       intakeMotorSubsystem.runIntakeMotors(0);
       pitchMotorSubsystem.runPitchMotor(0);
       swerveSubsystem.stopModules();
       System.out.println("interrupted");
    }

    public boolean isFinished() {
        return overideTimer.hasElapsed(2.5) || timerTwo.hasElapsed(0.3);
    }

}
