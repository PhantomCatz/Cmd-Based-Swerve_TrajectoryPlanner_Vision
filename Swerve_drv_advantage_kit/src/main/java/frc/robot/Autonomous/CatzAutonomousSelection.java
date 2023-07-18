package frc.robot.Autonomous;

import java.security.DrbgParameters.Reseed;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;
import org.littletonrobotics.junction.networktables.LoggedDashboardString;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.CatzConstants;
import frc.robot.CatzConstants.AutonomousPath;
import frc.robot.Utils.CatzStateUtil.MechanismState;
import frc.robot.commands.MechanismCmds.StateOrganizerCmd;
import frc.robot.subsystems.Arm.CatzArmSubsystem;
import frc.robot.subsystems.Elevator.CatzElevatorSubsystem;
import frc.robot.subsystems.Intake.CatzIntakeSubsystem;

public class CatzAutonomousSelection 
{
    private final CatzElevatorSubsystem elevator = CatzElevatorSubsystem.getInstance();
    private final CatzArmSubsystem arm = CatzArmSubsystem.getInstance();
    private final CatzIntakeSubsystem intake = CatzIntakeSubsystem.getInstance();

    public final LoggedDashboardChooser<Enum> chosenAllianceColor = new LoggedDashboardChooser<>("/alliance selector");
    public final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("/Auto Routine");


    public CatzAutonomousSelection()
    {
        chosenAllianceColor.addDefaultOption("Blue Alliance", CatzConstants.AllianceColor.BlUE_ALLIANCE);
        chosenAllianceColor.addOption       ("Red Alliance",  CatzConstants.AllianceColor.RED_ALLIANCE);
        SmartDashboard.putData              ("chosen allance",(Sendable) chosenAllianceColor);

        autoChooser.addDefaultOption("Do Nothing", new InstantCommand());

        autoChooser.addOption       ("TEST PATH",  new BalanceCmd());

        SmartDashboard.putData     ("chosenpath",(Sendable) autoChooser.get());
        
    }

    public Command TestPath()
    {
        new SequentialCommandGroup(
            
                new BalanceCmd(),
                new StateOrganizerCmd(elevator, arm, intake, MechanismState.PICKUPGROUND),
                new ParallelCommandGroup(
                    new StateOrganizerCmd(elevator, arm, intake, MechanismState.ScoreHigh),
                    new StateOrganizerCmd(elevator, arm, intake, MechanismState.STOW)
                )
            );
            return null;
    }



}
