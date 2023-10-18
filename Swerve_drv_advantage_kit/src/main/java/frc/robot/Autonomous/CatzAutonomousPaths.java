package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.CatzConstants;
import frc.robot.CatzConstants.AutonomousPath;
import frc.robot.Utils.CatzStateUtil;
import frc.robot.Utils.CatzStateUtil.GamePieceState;
import frc.robot.Utils.CatzStateUtil.SetMechanismState;
import frc.robot.commands.SetStateCmds.StateMachineCmd;
import frc.robot.subsystems.Arm.CatzArmSubsystem;
import frc.robot.subsystems.Elevator.CatzElevatorSubsystem;
import frc.robot.subsystems.Intake.CatzIntakeSubsystem;
import frc.robot.subsystems.drivetrain.CatzDriveTrainSubsystem;

public class CatzAutonomousPaths 
{
    private final static CatzElevatorSubsystem elevator = CatzElevatorSubsystem.getInstance();
    private final static CatzArmSubsystem arm = CatzArmSubsystem.getInstance();
    private final static CatzIntakeSubsystem intake = CatzIntakeSubsystem.getInstance();
    private final static CatzDriveTrainSubsystem driveTrain = CatzDriveTrainSubsystem.getInstance();
       
    CatzAutonomousPaths()
    {
        
    }

    public static Command testPath()
    {
        return null;
    }

    public static SequentialCommandGroup parallelScoreCube()
    {
        return new SequentialCommandGroup(
                    Commands.runOnce(() -> CatzStateUtil.newGamePieceState(GamePieceState.CUBE)),
                    new StateMachineCmd(SetMechanismState.SCORE_HIGH)
                        .raceWith(Commands.waitSeconds(1.0)),

                    new ParallelCommandGroup(
                        new DriveToPoseCmd(), 
                        new SequentialCommandGroup(
                            new StateMachineCmd(SetMechanismState.STOW)
                                .raceWith(Commands.waitSeconds(1.0)),
                            new StateMachineCmd(SetMechanismState.PICKUP_GROUND)
                                .raceWith(Commands.waitSeconds(1.0)),
                            intakeSequenceCommandGroup())
                                            ),

                    new ParallelCommandGroup(
                        new DriveToPoseCmd(), 
                        new SequentialCommandGroup(
                            new StateMachineCmd(SetMechanismState.STOW)
                                .raceWith(Commands.waitSeconds(1.0)),
                            new StateMachineCmd(SetMechanismState.SCORE_HIGH)
                                .raceWith(Commands.waitSeconds(1.0)))
                                            ),
                    intakeSequenceCommandGroup(),
                    new StateMachineCmd(SetMechanismState.STOW)
                                         );       
    }

    public static SequentialCommandGroup RightScore1HighConeBalance()
    {
        return new SequentialCommandGroup(
                    Commands.runOnce(() -> CatzStateUtil.newGamePieceState(GamePieceState.CONE)),
                    new StateMachineCmd(SetMechanismState.SCORE_HIGH),

                    new DriveToPoseCmd()
            
        );
    }

    public static SequentialCommandGroup intakeSequenceCommandGroup()
    {
        return new SequentialCommandGroup(
                Commands.runOnce(() -> intake.intakeRollerFunctionIN()),
                Commands.waitSeconds(0.5),
                Commands.runOnce(() -> intake.intakeRollersOff())           
                                         );
    }
}
