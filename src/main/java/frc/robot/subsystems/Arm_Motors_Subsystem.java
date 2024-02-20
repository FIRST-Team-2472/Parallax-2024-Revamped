package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmMotorsConstants.*;
import frc.robot.Constants.SensorConstants;ase;
import frc.robot.Constants.ArmMotorsConstants.*;
import frc.robot.Constants.SensorConstants;

public class Arm_Motors_Subsystem extends SubsystemBase {
    private CANSparkMax pitchMotor = new CANSparkMax(PitchMotor.kPitchMotorId, MotorType.kBrushless);
    private CANSparkMax shooterTopMotor = new CANSparkMax(ShooterMotors.kTopShooterMotorId, MotorType.kBrushless);
    private CANSparkMax shooterBottomMotor = new CANSparkMax(ShooterMotors.kBottomShooterMotorId, MotorType.kBrushless);
    private CANSparkMax pushMotor = new CANSparkMax(PushMotor.kPushMotorId, MotorType.kBrushless);
    private CANSparkMax intakeTopMotor = new CANSparkMax(IntakeMotors.kTopIntakeMotorId, MotorType.kBrushless);
    private CANSparkMax intakeBottomMotor = new CANSparkMax(IntakeMotors.kBottomIntakeMotorId, MotorType.kBrushless);
    private PIDController pitchPIDController = new PIDController(PitchMotor.kPitchMotorKP, 0, 0);
    private AnalogEncoder pitchAnalogEncoder = new AnalogEncoder(PitchMotor.kPitchEncoderId);
    DigitalInput photoElectricSensor = new DigitalInput(SensorConstants.kPhotoElectricSensorID);


    public Arm_Motors_Subsystem() {
        resetPitchEncoder();

    }

    public void resetPitchEncoder() {
        pitchAnalogEncoder.reset();
        pitchAnalogEncoder.setDistancePerRotation(360);
    }

    public double getPitchEncoderDeg() {
        return pitchAnalogEncoder.getDistance();
    }

    public void runPitchMotor(double motorSpeed) {
        pitchMotor.set(motorSpeed);
    }

    public void runShooterMotors(double motorSpeed) {
        shooterTopMotor.set(-motorSpeed);
        shooterBottomMotor.set(motorSpeed);
    }

    public void runPushMotor(double motorSpeed) {
        pushMotor.set(motorSpeed);
    }

    public void runIntakeMotors(double motorSpeed) {
        intakeTopMotor.set(motorSpeed);
        intakeBottomMotor.set(-motorSpeed);
    }

    public void getPhotoElectricSensor(double motorSpeed) {
        return photoElectricSensor;
    }
}
