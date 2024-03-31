package frc.robot.subsystems.ArmSubsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmMotorsConstants.*;
import frc.robot.Constants.SensorConstants;

public class IntakeMotorSubsystem extends SubsystemBase {
    private CANSparkMax pushMotor = new CANSparkMax(PushMotor.kPushMotorId, MotorType.kBrushless);
    private CANSparkMax intakeTopMotor = new CANSparkMax(IntakeMotors.kTopIntakeMotorId, MotorType.kBrushless);
    private CANSparkMax intakeBottomMotor = new CANSparkMax(IntakeMotors.kBottomIntakeMotorId, MotorType.kBrushless);
    DigitalInput photoElectricSensor = new DigitalInput(SensorConstants.kPhotoElectricSensorID);

    public IntakeMotorSubsystem() {

        // make sure all of them have the same settings in case we grabbed one with presets
        pushMotor.restoreFactoryDefaults();
        intakeTopMotor.restoreFactoryDefaults();
        intakeBottomMotor.restoreFactoryDefaults();

        intakeTopMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kBrake);
        intakeBottomMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kBrake);
        pushMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kBrake);
    }

    @Override
    public void periodic() {}

    public void runPushMotor(double motorSpeed) {
        pushMotor.set(motorSpeed);
    }

    public void runIntakeMotors(double motorSpeed) {
        intakeTopMotor.set(motorSpeed);
        intakeBottomMotor.set(-motorSpeed);
    }

    public boolean getPhotoElectricSensor(){
        return !photoElectricSensor.get();
    }
    
}
