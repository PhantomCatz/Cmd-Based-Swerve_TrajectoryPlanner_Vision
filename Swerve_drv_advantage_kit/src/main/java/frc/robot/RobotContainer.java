/***
 * RobotContainer
 * @version 1.0
 * @author Kynam Lenghiem
 * 
 * This class is how the command scheduler(robot.java replacment) is configured
 * Configures:
 * -xbox controller triggers
 * -default commands
 * -instanciated mechanisms using singleton implementation
 * -sets up autonomous from CatzAtutonomouschooser
 ***/

 package frc.robot;


 import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
 import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
 import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Autonomous.BalanceCmd;
import frc.robot.Autonomous.CatzAutonomousSelection;
 import frc.robot.Utils.CatzStateUtil;
 import frc.robot.Utils.CatzStateUtil.GamePieceState;
 import frc.robot.Utils.CatzStateUtil.SetMechanismState;
import frc.robot.commands.StateMachineCmd;
import frc.robot.commands.TeleopDriveCmd;
import frc.robot.commands.ManualStateCmds.ArmManualCmd;
import frc.robot.commands.ManualStateCmds.ElevatorManualCmd;
import frc.robot.commands.ManualStateCmds.IntakeManualCmd;
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
    private static CatzDriveTrainSubsystem driveTrain;
    private static CatzElevatorSubsystem elevator;
    private static CatzIntakeSubsystem intake;
    private static CatzArmSubsystem arm;
    //private final CatzRobotTracker robotTracker; //TBD need to test and modify swerve drive code for this

    private final CatzAutonomousSelection auton = new CatzAutonomousSelection();

    //xbox controller
    private CommandXboxController xboxDrv;
    private CommandXboxController xboxAux;

    //RobotContainer Constants
    private final int XBOX_DRV_PORT = 0;
    private final int XBOX_AUX_PORT = 1;
 
       
    
 
   /** The container for the robot. Contains subsystems, OI devices, and commands. 
    *    -since multiple classes are referencing these mechansims, 
    *         mechanisms are instantiated inside mechanism class(singleton)
   */
   public RobotContainer() 
   {
    //instantiate subsystems
     driveTrain = CatzDriveTrainSubsystem.getInstance(); 
     elevator = CatzElevatorSubsystem.getInstance();
     arm = CatzArmSubsystem.getInstance();
     intake = CatzIntakeSubsystem.getInstance();

     NamedCommands.registerCommand("autoBalance", new BalanceCmd(driveTrain));
     NamedCommands.registerCommand("stateMachineHigh", new StateMachineCmd(SetMechanismState.SCORE_HIGH));

     

 
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
   //---------------------------------------Button mechanism cmds-----------------------------------------------------------------
     xboxAux.y().onTrue(new StateMachineCmd(SetMechanismState.SCORE_HIGH));
     xboxAux.b().onTrue(new StateMachineCmd(SetMechanismState.SCORE_MID));
     xboxAux.a().onTrue(new StateMachineCmd(SetMechanismState.SCORE_LOW));
     xboxAux.x().or(xboxDrv.rightStick())
                .onTrue(new StateMachineCmd(SetMechanismState.STOW));
     xboxAux.start().or(xboxDrv.leftStick())
                .onTrue(new StateMachineCmd(SetMechanismState.PICKUP_GROUND));
 
   
   //--------------------------------------------Manual Cmds---------------------------------------------------------------------------
     //arm
       
     xboxAux.rightTrigger()
     .onTrue(new ArmManualCmd(true,
                            false))
     .onFalse(Commands.runOnce(
             () -> arm.setArmPwr(0.0)
                              ));
 
     xboxAux.leftTrigger()
     .onTrue(new ArmManualCmd(false,
                            true))
     .onFalse(Commands.run(
             () -> arm.setArmPwr(0.0)
                          ));

    //intake
    xboxAux.leftStick().onTrue(new IntakeManualCmd(() -> xboxAux.getLeftY(), 
                                                   () -> xboxAux.leftStick().getAsBoolean()));
    //elevator     
     xboxAux.rightStick().onTrue(new ElevatorManualCmd(() -> xboxAux.getRightY(), 
                                                       () -> xboxAux.rightStick().getAsBoolean()));
  
 
 
   //-----------------------------------commands with no subsystem--------------------------------------------------------------------
     xboxAux.back()
     .onTrue(Commands.runOnce(
         () -> CatzStateUtil.newGamePieceState(GamePieceState.NONE)
                             ));
 
     xboxAux.povLeft()
     .onTrue(Commands.runOnce(
         () -> CatzStateUtil.newGamePieceState(GamePieceState.CUBE)
                             ));
 
     xboxAux.povRight()
     .onTrue(Commands.runOnce(
        () -> CatzStateUtil.newGamePieceState(GamePieceState.CONE)
                             ));
 
      //disabling softlimits only when both bumpers are pressed
      
     xboxAux.leftBumper().and(xboxAux.rightBumper()) 
     .onTrue(Commands.runOnce(() -> intake.softLimitOverideDisabled()))
     .onFalse(Commands.runOnce(() -> intake.softLimitOverideEnabled()));
 
 
     xboxDrv.start().onTrue(Commands.runOnce(() -> driveTrain.zeroGyro()));
 
     //xboxDrv.b().onTrue(Commands.runOnce(() -> driveTrain.lockWheels())); TBD need to add this back in
 
     
     //--------------------------Intake Rollers--------------------------
       xboxAux.rightBumper().onTrue(Commands.runOnce(() -> intake.intakeRollerFunctionIN()))
                            .onFalse(Commands.runOnce(() -> intake.intakeRollersOff()));
 
       xboxAux.leftBumper().onTrue(Commands.runOnce(
         () -> {
           intake.intakeRollerFunctionOUT();
         })).onFalse(Commands.runOnce(
           () -> {
             intake.intakeRollersOff();
           }));
           
 
   }
   //mechanisms with default commands revert back to these cmds if no other cmd requiring the subsystem is active
   //
   private void defaultCommands() 
   {  
      driveTrain.setDefaultCommand(new TeleopDriveCmd(() -> xboxDrv.getLeftX(),
                                                      () -> xboxDrv.getLeftY(),
                                                      () -> xboxDrv.getRightX(),
                                                      () -> xboxDrv.getRightTriggerAxis(), 
                                                      () -> xboxDrv.b().getAsBoolean()));
                                                     
 
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