package frc.robot.subsystems.ArmSubsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmMotorsConstants.*;

public class ShootingMotorSubsystem extends SubsystemBase {
    private static CANSparkMax shooterTopMotor = new CANSparkMax(ShooterMotors.kTopShooterMotorId, MotorType.kBrushless);
    private CANSparkMax shooterBottomMotor = new CANSparkMax(ShooterMotors.kBottomShooterMotorId, MotorType.kBrushless);


    public ShootingMotorSubsystem() {

        // make sure all of them have the same settings in case we grabbed one with presets
        shooterTopMotor.restoreFactoryDefaults();
        shooterBottomMotor.restoreFactoryDefaults();
        shooterBottomMotor.setSmartCurrentLimit(39);
        shooterTopMotor.setSmartCurrentLimit(39);

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter speed", getShooterSpeed());
    }

    public void runShooterMotors(double motorSpeed) {
        shooterTopMotor.set(-motorSpeed);
        shooterBottomMotor.set(motorSpeed);
    }

    public double getShooterSpeed(){
        return shooterTopMotor.getEncoder().getVelocity();
    }
}
