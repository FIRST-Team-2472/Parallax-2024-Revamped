package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoAiming;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ArmSubsystems.IntakeMotorSubsystem;
import frc.robot.subsystems.ArmSubsystems.PitchMotorSubsystem;
import frc.robot.subsystems.ArmSubsystems.ShootingMotorSubsystem;

public class FastAutoAimCmd extends Command {
    PitchMotorSubsystem pitchSubsystem;
    SwerveSubsystem swerveSubsystem;
    ShootingMotorSubsystem shootingSubsystem;
    IntakeMotorSubsystem intakeSubsystem;

    public FastAutoAimCmd(PitchMotorSubsystem pitchMotorSubsystem, SwerveSubsystem swerveSubsystem,
            ShootingMotorSubsystem shootingSubsystem, IntakeMotorSubsystem intakeSubsystem) {
        this.pitchSubsystem = pitchMotorSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        this.shootingSubsystem = shootingSubsystem;
        this.intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void initialize() {
        Pose2d botPose = swerveSubsystem.getPose();

        new SequentialCommandGroup(
            new ParallelDeadlineGroup(
                new ParallelCommandGroup(
                    new SetArmPitchCmd(pitchSubsystem, AutoAiming.getPitch(botPose)),
                    new SwerveRotateToAngle(swerveSubsystem, Rotation2d.fromDegrees(AutoAiming.getYaw(botPose)))
                ),
                new RunShooterCmd(shootingSubsystem, 4000)
            ),
            new ShootNoteCmd(shootingSubsystem, intakeSubsystem, 0.9, 4000)
        ).schedule();

    }

    public boolean isFinished() {
        return true;
    }

}
