package frc.robot.commands;
import frc.robot.subsystems.Garra;
import frc.robot.subsystems.Hombro;
import frc.robot.subsystems.Brazo;
import frc.robot.subsystems.driveTrain.DriveTrainSubsystemModified;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class BrakeToggleCommand extends CommandBase{
    
    private Garra garra;
    private Hombro hombro;
    private Brazo brazo;
    private DriveTrainSubsystemModified dTrain;
    private Boolean brakeFlag;


    public BrakeToggleCommand(Garra garra, Hombro hombro, Brazo brazo, DriveTrainSubsystemModified dTrain){
        this.garra = garra;
        this.hombro = hombro;
        this.brazo = brazo;
        this.dTrain = dTrain;
        brakeFlag = true;
    }

    public void initialize(){
        if(brakeFlag){
            dTrain.setCoast();
            hombro.setCoast();
            garra.setCoast();
            brazo.setCoast();
            brakeFlag = false;
        } else{
            dTrain.setBrake();
            hombro.setBrake();
            garra.setBrake();
            brazo.setCoast();
            brakeFlag = true;
        }
    }

    public boolean isFinished(){
        return true;
    }

}
