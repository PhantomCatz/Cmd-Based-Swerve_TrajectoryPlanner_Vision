// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Utils.CatzStateUtil;
import frc.robot.Utils.CatzStateUtil.GamePieceState;
import frc.robot.Utils.CatzStateUtil.MechanismState;
import frc.robot.commands.MechanismCmds.MechanismCommand;
import frc.robot.subsystems.Arm.CatzArmSubsystem;
import frc.robot.subsystems.Elevator.CatzElevatorSubsystem;
import frc.robot.subsystems.Intake.CatzIntakeSubsytem;
import frc.robot.subsystems.drivetrain.CatzDriveTrainSubsystem;
import frc.robot.subsystems.drivetrain.ModuleIO;
import frc.robot.subsystems.drivetrain.ModuleIOReal;
import frc.robot.subsystems.vision.CatzRobotTracker;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
      private final CatzDriveTrainSubsystem driveTrain;
      private final CatzElevatorSubsystem elevator;
      //private final CatzIntakeSubsytem intake;
      private final CatzArmSubsystem arm;
      //private final CatzRobotTracker robotTracker;
      private final CatzStateUtil stateUtil;
      
      private CommandXboxController xboxDrv;
      private CommandXboxController xboxAux;
   
      //RobotContainer Constants
      private final int XBOX_DRV_PORT = 0;
      private final int XBOX_AUX_PORT = 1;

      
   

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() 
  {
    driveTrain = CatzDriveTrainSubsystem.getInstance();
    elevator = CatzElevatorSubsystem.getInstance();
    arm = CatzArmSubsystem.getInstance();
    stateUtil = new CatzStateUtil();

    xboxDrv = new CommandXboxController(XBOX_DRV_PORT); 
    xboxAux = new CommandXboxController(XBOX_AUX_PORT);


    // Configure the trigger bindings
    defaultCommands();
    configureBindings();
  }

  
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() 
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    xboxAux.y().onTrue(new MechanismCommand(elevator, arm, null, MechanismState.ScoreHigh));

    

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

  }
  private void defaultCommands() 
  { 
    driveTrain.setDefaultCommand(new RunCommand(() -> driveTrain.cmdProcSwerve(xboxDrv.getLeftX(), 
                                                                     xboxDrv.getLeftY(), 
                                                                     xboxDrv.getRightX(),
                                                                     driveTrain.getGyroAngle(),  
                                                                     xboxDrv.getRightTriggerAxis()), driveTrain));
  
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
  
}
