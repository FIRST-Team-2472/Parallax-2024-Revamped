package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmMotorsSubsystem;
import frc.robot.Constants.ArmMotorsConstants.Encoder;

public class ArmMotorsCmd extends Command{
    private Supplier<Double> pitchMotor;
    private Double intakeMotorsSpeed, shooterMotorsSpeed, pushMotorSpeed, pitchMotorSpeed;
    private Supplier<Boolean> intakeMotorsRunning, shooterMotorsSpeaker, shooterMotorsAmp, pushMotorRunning;
    private ArmMotorsSubsystem armSubsystem;
    private AnalogEncoder encoder;
    public ArmMotorsCmd(ArmMotorsSubsystem armSubsystem, AnalogEncoder encoder, Supplier<Double> pitchMotor, Supplier<Boolean> shooterMotorsSpeaker, Supplier<Boolean> shooterMotorsAmp, 
        Supplier<Boolean> pushMotorRunning, Supplier<Boolean> intakeMotorsRunning){
        this.pitchMotor = pitchMotor;
        /**
        this.shooterMotorsSpeaker = shooterMotorsSpeaker;
        this.shooterMotorsAmp = shooterMotorsAmp;
        this.pushMotorRunning = pushMotorRunning;
        this.intakeMotorsRunning = intakeMotorsRunning;
        */
        this.armSubsystem = armSubsystem;
        this.encoder = encoder;
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
        pitchMotorSpeed = pitchMotorSpeed > 0.5 ? 0.5 : pitchMotorSpeed;
        pitchMotorSpeed = pitchMotorSpeed < -0.5 ? -0.5 : pitchMotorSpeed;
        if (encoder.getDistance() >= Encoder.kEncoderPositiveLimit || encoder.getDistance() <= Encoder.kEncoderNegativeLimit)
            pitchMotorSpeed = 0.0;
        /**
        shooterMotorsSpeed = shooterMotorsSpeaker.get() ? 0.5 : 0;
        shooterMotorsSpeed = shooterMotorsAmp.get() ? 0.25 : 0;
        pushMotorSpeed = pushMotorRunning.get() ? 0.5 : 0;
        intakeMotorsSpeed = intakeMotorsRunning.get() ? 0.5 : 0;
        */
        // if (pitchMotorSpeed > 0.5) pitchMotorSpeed = 0.5;
        // if (pitchMotorSpeed < -0.5) pitchMotorSpeed = -0.5;
        // if (shooterMotorsRunning.get()) shooterMotorsSpeed = 0.5;
        // if (pushMotorRunning.get()) pushMotorSpeed = 0.5;
        // if (intakeMotorsRunning.get()) intakeMotorsSpeed = 0.5;
        armSubsystem.runPitchMotor(pitchMotorSpeed);
        /**
        armSubsystem.runShooterMotors(shooterMotorsSpeed);
        armSubsystem.runPushMotor(pushMotorSpeed);
        armSubsystem.runIntakeMotors(intakeMotorsSpeed);
        */
        System.out.println(encoder.getDistance());
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
