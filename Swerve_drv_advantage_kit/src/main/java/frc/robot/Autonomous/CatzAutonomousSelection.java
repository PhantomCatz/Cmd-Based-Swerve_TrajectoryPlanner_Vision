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



public class CatzAutonomousSelection 
{

    public final LoggedDashboardChooser<Enum> chosenAllianceColor = new LoggedDashboardChooser<>("/alliance selector");
    public final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("/Auto Routine");

    public CatzAutonomousPaths autonPaths = new CatzAutonomousPaths();
    public CatzAutonomousSelection()
    {
        chosenAllianceColor.addDefaultOption("Blue Alliance", CatzConstants.AllianceColor.BlUE_ALLIANCE);
        chosenAllianceColor.addOption       ("Red Alliance",  CatzConstants.AllianceColor.RED_ALLIANCE);


        autoChooser.addDefaultOption("Do Nothing", new InstantCommand());

        autoChooser.addOption       ("TEST PATH",  CatzAutonomousPaths.testPath());
        autoChooser.addOption       ("Parallel Score Cube", CatzAutonomousPaths.parallelScoreCube());
        autoChooser.addOption       ("RightScore1HighConeBalance", CatzAutonomousPaths.RightScore1HighConeBalance());

    }





    
    



}
