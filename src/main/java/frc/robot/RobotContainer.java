package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

public class RobotContainer {
  Arm_Motors_Subsystem armSubsystem = new Arm_Motors_Subsystem();
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  XboxController xbox = new XboxController(OperatorConstants.kDriverControllerPort);
  public static Joystick leftJoystick = new Joystick(OperatorConstants.kLeftJoyPort);
  public static Joystick rightJoystick = new Joystick(OperatorConstants.kRightJoyPort);
  public RobotContainer() {
    armSubsystem.setDefaultCommand(new ArmMotorsCmd(armSubsystem, () -> xbox.getLeftY(), // Pitch Motor
      () -> xbox.getLeftTriggerAxis() > 0.5, () -> xbox.getLeftBumper(), // Shooter Motors
        () -> xbox.getRightTriggerAxis() > 0.5, // Push Motor
          () -> xbox.getRightBumper())); // Intake Motors

    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(swerveSubsystem, 
    ()-> leftJoystick.getY(),
     ()-> -leftJoystick.getX(),
      ()-> -rightJoystick.getX(),
       ()-> rightJoystick.getRawButton(2)));
    configureBindings();
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
