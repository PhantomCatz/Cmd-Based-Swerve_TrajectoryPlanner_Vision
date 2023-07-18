package frc.robot.Autonomous;

import java.security.DrbgParameters.Reseed;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;
import org.littletonrobotics.junction.networktables.LoggedDashboardString;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.CatzConstants;
import frc.robot.CatzConstants.AutonomousPath;
import frc.robot.Utils.CatzStateUtil;
import frc.robot.Utils.CatzStateUtil.GamePieceState;
import frc.robot.Utils.CatzStateUtil.MechanismState;
import frc.robot.commands.SetStateCmd;
import frc.robot.subsystems.Arm.CatzArmSubsystem;
import frc.robot.subsystems.Elevator.CatzElevatorSubsystem;
import frc.robot.subsystems.Intake.CatzIntakeSubsystem;
import frc.robot.subsystems.drivetrain.CatzDriveTrainSubsystem;


public class CatzAutonomousSelection 
{
    private final CatzElevatorSubsystem elevator = CatzElevatorSubsystem.getInstance();
    private final CatzArmSubsystem arm = CatzArmSubsystem.getInstance();
    private final CatzIntakeSubsystem intake = CatzIntakeSubsystem.getInstance();
    private final CatzDriveTrainSubsystem driveTrain = CatzDriveTrainSubsystem.getInstance();

    public final LoggedDashboardChooser<Enum> chosenAllianceColor = new LoggedDashboardChooser<>("/alliance selector");
    public final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("/Auto Routine");


    public CatzAutonomousSelection()
    {
        chosenAllianceColor.addDefaultOption("Blue Alliance", CatzConstants.AllianceColor.BlUE_ALLIANCE);
        chosenAllianceColor.addOption       ("Red Alliance",  CatzConstants.AllianceColor.RED_ALLIANCE);


        autoChooser.addDefaultOption("Do Nothing", new InstantCommand());

        autoChooser.addOption       ("TEST PATH",  TestPath());
        autoChooser.addOption       ("Parallel Score Cube", parallelScoreCube());


        
    }

    public Command TestPath()
    {
        return null;
    }

    public Command parallelScoreCube()
    {
        new SequentialCommandGroup(
                Commands.runOnce(() -> {CatzStateUtil.newGamePieceState(GamePieceState.CUBE);}),
                new SetStateCmd(elevator, arm, intake, MechanismState.SCORE_LOW),
                new ParallelCommandGroup(
                    new DriveToPoseCmd(), 
                    new SequentialCommandGroup(
                                            new SetStateCmd(elevator, arm, intake, MechanismState.STOW),
                                            new SetStateCmd(elevator, arm, intake, MechanismState.PICKUP_GROUND),
                                            Commands.run(() -> {intake.intakeRollerFunctionIN();}, intake))
                                        ),
                Commands.runOnce(() -> {Timer.delay(0.5);}),
                new ParallelCommandGroup(
                    Commands.run(() -> {intake.intakeRollersOff();}, intake),
                    new DriveToPoseCmd(), 
                    new SequentialCommandGroup(
                                            new SetStateCmd(elevator, arm, intake, MechanismState.STOW),
                                            new SetStateCmd(elevator, arm, intake, MechanismState.SCORE_HIGH))
                                                ),
                Commands.run(() -> {intake.intakeRollerFunctionOUT();}, intake),
                Commands.runOnce(() -> {Timer.delay(1);}),
                Commands.run(() -> {intake.intakeRollersOff();}, intake),
                new SetStateCmd(elevator, arm, intake, MechanismState.STOW)
                );
            return null;
    }



}
