package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmMotorsConstants.*;

public class ArmMotorsSubsystem extends SubsystemBase {
    private CANSparkMax pitchMotor = new CANSparkMax(PitchMotor.kPitchMotorId, MotorType.kBrushless);
    private CANSparkMax shooterTopMotor = new CANSparkMax(ShooterMotors.kTopShooterMotorId, MotorType.kBrushless);
    private CANSparkMax shooterBottomMotor = new CANSparkMax(ShooterMotors.kBottomShooterMotorId, MotorType.kBrushless);
    private CANSparkMax pushMotor = new CANSparkMax(PushMotor.kPushMotorId, MotorType.kBrushless);
    private CANSparkMax intakeTopMotor = new CANSparkMax(IntakeMotors.kTopIntakeMotorId, MotorType.kBrushless);
    private CANSparkMax intakeBottomMotor = new CANSparkMax(IntakeMotors.kBottomIntakeMotorId, MotorType.kBrushless);
    private PIDController pitchPIDController = new PIDController(PitchMotor.kPitchMotorKP, 0, 0);
<<<<<<< HEAD:src/main/java/frc/robot/subsystems/ArmMotorsSubsystem.java
=======
    private AnalogEncoder pitchAnalogEncoder = new AnalogEncoder(PitchMotor.kPitchEncoderId);

    public Arm_Motors_Subsystem() {
        resetPitchEncoder();
        //pitchMotor.getEncoder().setPosition(getPitchEncoderDeg() * "Conversion Factor");
        pitchMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) PitchMotor.kPitchEncoderReverseLimit);
        pitchMotor.setSoftLimit(SoftLimitDirection.kForward, (float) PitchMotor.kPitchEncoderForwardLimit);
    }

    public void resetPitchEncoder() {
        // pitchAnalogEncoder.reset();
        pitchAnalogEncoder.setDistancePerRotation(360);
    }

    public double getPitchEncoderDeg() {
        return pitchAnalogEncoder.getDistance();
    }
>>>>>>> Encoder-and-KP:src/main/java/frc/robot/subsystems/Arm_Motors_Subsystem.java

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

<<<<<<< HEAD:src/main/java/frc/robot/subsystems/ArmMotorsSubsystem.java
} 
    
=======
    public void runPitchMotorWithKP(double angleDeg) {

        double speed = pitchPIDController.calculate(getPitchEncoderDeg(), angleDeg);
        
        runPitchMotor(speed);
    }
}
>>>>>>> Encoder-and-KP:src/main/java/frc/robot/subsystems/Arm_Motors_Subsystem.java
