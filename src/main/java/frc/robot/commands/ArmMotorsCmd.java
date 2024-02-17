package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SensorConstants;
import frc.robot.subsystems.ArmMotorsSubsystem;

public class ArmMotorsCmd extends Command{
    private Supplier<Double> pitchMotor;
    private Double intakeMotorsSpeed, shooterMotorsSpeed, pushMotorSpeed, pitchMotorSpeed;
    private Supplier<Boolean> intakeMotorsRunning, shooterMotorsSpeaker, shooterMotorsAmp, pushMotorRunning;
    private ArmMotorsSubsystem armSubsystem;
    private AnalogEncoder pitchMotorEncoder;
    DigitalInput photoElectricSensor = new DigitalInput(SensorConstants.kPhotoElectricSensorID);
    public ArmMotorsCmd(ArmMotorsSubsystem armSubsystem, AnalogEncoder pitchMotorEncoder, Supplier<Double> pitchMotor, Supplier<Boolean> shooterMotorsSpeaker, Supplier<Boolean> shooterMotorsAmp, 
        Supplier<Boolean> pushMotorRunning, Supplier<Boolean> intakeMotorsRunning){
        this.pitchMotor = pitchMotor;
        this.shooterMotorsSpeaker = shooterMotorsSpeaker;
        this.shooterMotorsAmp = shooterMotorsAmp;
        this.pushMotorRunning = pushMotorRunning;
        this.intakeMotorsRunning = intakeMotorsRunning;
        this.armSubsystem = armSubsystem;
        this.pitchMotorEncoder = pitchMotorEncoder;
        addRequirements(armSubsystem);
    } 
    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        pitchMotorSpeed = pitchMotor.get();
        if (pitchMotorSpeed < 0.1 && pitchMotorSpeed > -0.1) pitchMotorSpeed = 0.0;
        pitchMotorSpeed *= 0.3;
        shooterMotorsSpeed = shooterMotorsSpeaker.get() ? .75 : (shooterMotorsAmp.get() ? 0.5 : 0);
        pushMotorSpeed = pushMotorRunning.get() ? 0.5 : (intakeMotorsRunning.get() && !photoElectricSensor.get() ? 0.2 : 0);
        intakeMotorsSpeed = intakeMotorsRunning.get() && !photoElectricSensor.get() ? 1.0 : 0;
        armSubsystem.runPitchMotor(pitchMotorSpeed);
        armSubsystem.runShooterMotors(shooterMotorsSpeed);
        armSubsystem.runPushMotor(pushMotorSpeed);
        armSubsystem.runIntakeMotors(intakeMotorsSpeed);
        System.out.println(pitchMotorEncoder.getDistance());
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
