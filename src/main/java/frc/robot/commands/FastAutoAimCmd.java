package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AimPoint;
import frc.robot.AutoAiming;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ArmSubsystems.IntakeMotorSubsystem;
import frc.robot.subsystems.ArmSubsystems.PitchMotorSubsystem;
import frc.robot.subsystems.ArmSubsystems.ShootingMotorSubsystem;

public class FastAutoAimCmd extends Command {

    public FastAutoAimCmd() {
        PIDController hg = new PIDController(1, 1, 1);
    }

    @Override
    public void initialize() {


    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {

    }

    public boolean isFinished() {
        return false;
    }

}
