package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.Limelight;
// import frc.lib.util.LimelightTrajectory;
import frc.robot.Constants;

public class Vision extends SubsystemBase{
    
    Swerve swerve; 
    public Vision(Swerve swerve){
        Limelight.init("limelight-top");
        this.swerve = swerve; 

    }

    public void trackTag(){
        // RobotContainer.s_Swerve.rotateToAngle(180);
        Limelight.updateData();
        Limelight.setPipeline(0);
        

        if (Limelight.getTv() == 1 && Limelight.getTa() > Constants.Vision.threshold_distance) {
            double y_vel;
            double x_vel;
            if (Math.abs(Limelight.getTx()) > Constants.Vision.threshold_tx_offset) {
                y_vel = -1 * Limelight.getTx() * 0.06;
                x_vel = 0 * (Math.sin(Math.PI / Constants.Vision.y_vel_constant) * Limelight.getTx());
            } 
            
            else {
                x_vel = 0;
                y_vel = 0;
            }
      
            Translation2d velocity = new Translation2d(x_vel, y_vel);
            if (swerve.isLowGear()) {
                velocity = velocity.times(5);
            }

            System.out.println("### DRIVING ###");
            swerve.drive(velocity, 0, true, false);
        } 
        else if (Limelight.getTv() == 0) {
            swerve.stop();
        }
        
    
    }

}