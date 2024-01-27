package frc.lib.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight {

    //create NetworkTable objects
    private static NetworkTableInstance nInstance;
    private static NetworkTable table; 
    private static final SendableChooser<Integer> m_chooser = new SendableChooser<Integer>(); 
    //limelight values
    private static NetworkTableEntry ta; 
    private static NetworkTableEntry tv; 
    private static NetworkTableEntry ty;
    private static NetworkTableEntry tx; 
    private static NetworkTableEntry tid; 
    private static NetworkTableEntry t6t_cs; 

    //pipelines
    private static int current_pipeline = 0; 
    
    public static void init(String key) {
        nInstance = NetworkTableInstance.getDefault();

        table = nInstance.getTable(key);
    

        m_chooser.setDefaultOption("tag 1", 1);
        m_chooser.addOption("tag 2", 2);
        m_chooser.addOption("tag 3", 3);
        m_chooser.addOption("tag 4", 4);
        m_chooser.addOption("tag 5", 5);
        m_chooser.addOption("tag 6", 6);
        m_chooser.addOption("tag 7", 7);
        m_chooser.addOption("tag 8", 8);

        //SmartDashboard.putData("Pipeline", m_chooser);

        ta = table.getEntry("ta");
        tv = table.getEntry("tv");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        tid = table.getEntry("tid");
        t6t_cs = table.getEntry("targetpose_cameraspace");

        //m_tagSelected = m_chooser.getSelected();
        //current_pipeline = m_tagSelected; 
        table.getEntry("pipeline").setNumber(current_pipeline); 
    
    }

    public static void setPipeline(int newPipeline) {
        current_pipeline = newPipeline;
        table.getEntry("pipeline").setNumber(newPipeline); 

    }

    public static double getTa() {
        return ta.getDouble(0); 
    }

    public static double getTv() {
        return tv.getDouble(0); 
    }

    public static double getTx() {
        return tx.getDouble(0); 
    }

    public static double getTy() {
        return ty.getDouble(0); 
    }

    public static double getTid() {
        return tid.getDouble(-1); 
    }

    public static double[] getT6T_CS() {
        return t6t_cs.getDoubleArray(new double[6]);
    }

    public static double getTagDistance() {
        return Math.abs(t6t_cs.getDoubleArray(new double[6])[2]);
    }

    public static double getPipeline(){ 
        return current_pipeline; 
    }

    public static void updateData() {
        ta = table.getEntry("ta");
        tv = table.getEntry("tv");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        tid = table.getEntry("tid");
        t6t_cs = table.getEntry("targetpose_cameraspace");
        SmartDashboard.putNumber("Tag Distance", getTagDistance());
    }
    
    /*
    public static void printTagDistance(Double ta) { 
        //using ta area from limelight for apriltag, scaled proportion for distance in meters
        double distance = ta / Constants.proportion; //ta = 20
        SmartDashboard.putNumber("Tag distance", distance);
        System.out.println("Tag " + getTid() + " is " + distance + " meters away");
    }

    */

}
