// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Autonomous.CatzAutonomousSelection;
import frc.robot.Utils.CatzStateUtil;
import frc.robot.Utils.CatzStateUtil.GamePieceState;
import frc.robot.Utils.CatzStateUtil.MechanismState;
import frc.robot.commands.SetStateCmd;
import frc.robot.commands.TeleopDriveCmd;
import frc.robot.commands.MechanismCmds.ArmCmd;
import frc.robot.commands.MechanismCmds.ElevatorCmd;
import frc.robot.commands.MechanismCmds.IntakeCmd;
import frc.robot.subsystems.Arm.CatzArmSubsystem;
import frc.robot.subsystems.Elevator.CatzElevatorSubsystem;
import frc.robot.subsystems.Intake.CatzIntakeSubsystem;
import frc.robot.subsystems.drivetrain.CatzDriveTrainSubsystem;


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
      private final CatzIntakeSubsystem intake;
      private final CatzArmSubsystem arm;
      //private final CatzRobotTracker robotTracker;

      private final CatzAutonomousSelection auton = new CatzAutonomousSelection();

      
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
    intake = CatzIntakeSubsystem.getInstance();


    xboxDrv = new CommandXboxController(XBOX_DRV_PORT); 
    xboxAux = new CommandXboxController(XBOX_AUX_PORT);


    // Configure the trigger bindings and default cmds
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
  //---------------------------------------Aux button mechanism cmds-----------------------------------------------------
    xboxAux.y().onTrue(new SetStateCmd(elevator, arm, intake, MechanismState.SCORE_HIGH));
    xboxAux.b().onTrue(new SetStateCmd(elevator, arm, intake, MechanismState.SCORE_MID));
    xboxAux.a().onTrue(new SetStateCmd(elevator, arm, intake, MechanismState.SCORE_LOW));
    xboxAux.x().or(xboxDrv.rightStick()).onTrue(new SetStateCmd(elevator, arm, intake, MechanismState.STOW));
    xboxAux.start().or(xboxDrv.leftStick()).onTrue(new SetStateCmd(elevator, arm, intake, MechanismState.PICKUP_GROUND));
    
  //------------------------------------------Drive button Mechanism cmds---------------------------------------
    xboxDrv.rightStick().onTrue(new SetStateCmd(elevator, arm, intake, MechanismState.STOW));
    xboxDrv.leftStick().onTrue(new SetStateCmd(elevator, arm, intake, MechanismState.PICKUP_GROUND));

  //--------------------------------------------Manual Cmds---------------------------------------------------------------------------
    //arm
    xboxAux.rightTrigger().onTrue(new ArmCmd(arm, CatzStateUtil.ArmState.MANUAL, null, true, false))
                          .onFalse(Commands.run(
                            () -> {
                            arm.setArmPwr(0.0);
                            }));
    xboxAux.leftTrigger().onTrue(new ArmCmd(arm, CatzStateUtil.ArmState.MANUAL, null, false, true))
                         .onFalse(Commands.run(
                            () -> {
                            arm.setArmPwr(0.0);
                            }));

  //-----------------------------------commands with no subsystem----------------------------
    xboxAux.back().onTrue(Commands.runOnce(
      () -> {
      CatzStateUtil.newGamePieceState(GamePieceState.NONE);
      }));

    xboxAux.povLeft().onTrue(Commands.runOnce(
      () -> {
        CatzStateUtil.newGamePieceState(GamePieceState.CUBE);
      }));

    xboxAux.povRight().onTrue(Commands.runOnce(
      () -> {
        CatzStateUtil.newGamePieceState(GamePieceState.CONE);
      }));

    xboxAux.leftBumper().and(xboxAux.rightBumper())  //disabling softlimits only when both bumpers are pressed
    .onTrue(Commands.runOnce(
    () -> {
      intake.softLimitOverideDisabled();
    }))
    .onFalse(Commands.runOnce(
    () -> {
      intake.softLimitOverideEnabled();
    }));


    xboxDrv.start().onTrue(Commands.runOnce(
      () -> {
        driveTrain.zeroGyro();
      }));

    xboxDrv.b().onTrue(Commands.runOnce(
      () -> {
        driveTrain.lockWheels();
      }));

    //--------------------------Intake Rollers--------------------------
      xboxAux.rightBumper().onTrue(Commands.run(
        () -> {
          intake.intakeRollerFunctionIN();
        })).onFalse(Commands.runOnce(
          () -> {
          intake.intakeRollersOff();
          }));

      xboxAux.leftBumper().onTrue(Commands.run(
        () -> {
          intake.intakeRollerFunctionOUT();
        })).onFalse(Commands.runOnce(
          () -> {
            intake.intakeRollersOff();
          }));




  }
  private void defaultCommands() 
  { 
    driveTrain.setDefaultCommand(new TeleopDriveCmd(driveTrain, () -> xboxDrv.getLeftX(),
                                                                () -> xboxDrv.getLeftY(),
                                                                () -> xboxDrv.getRightX(),
                                                                () -> xboxDrv.getRightTriggerAxis()));
    intake.setDefaultCommand(new IntakeCmd(intake, CatzStateUtil.IntakeState.MANUAL, null, () -> xboxAux.getLeftX(), () -> xboxAux.leftStick().getAsBoolean()));     

    elevator.setDefaultCommand(new ElevatorCmd(elevator, CatzStateUtil.ElevatorState.MANUAL, null,
                                                () -> xboxAux.getRightY(), 
                                                () -> xboxAux.rightStick().getAsBoolean()));

  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return auton.autoChooser.get();
  }
  
}
