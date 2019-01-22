package frc.robot.motion;

import java.util.HashMap;
import java.util.Map;

import frc.robot.motion.Path.Mode;


public class PathList {
	
	 private static Map<String, Path> paths;
		 /**
		 * 
		 */
	 static {
		 paths = new HashMap<String, Path>();
		 paths.put("StartLSwitchL", new Path("StartLeftToSwitchLeft", Mode.BACKWARD_HIGH));
		 paths.put("StartLSwitchR", new Path("StartLeftToSwitchRight", Mode.BACKWARD_HIGH));
		 paths.put("StartRSwitchL", new Path("StartRightToSwitchLeft", Mode.BACKWARD_HIGH));
		 paths.put("StartRSwitchR", new Path("StartRightToSwitchRight", Mode.BACKWARD_HIGH));
		 paths.put("StartCSwitchL", new Path("StartCenterToSwitchLeft", Mode.FORWARD_HIGH));
		 paths.put("StartCSwitchR", new Path("StartCenterToSwitchRight", Mode.FORWARD_HIGH));
		 paths.put("StartRScaleR", new Path("StartRightToScaleRight", Mode.BACKWARD_HIGH));
		 paths.put("StartRScaleL", new Path("StartRightToScaleLeft", Mode.BACKWARD_HIGH));
		 paths.put("StartLScaleR", new Path("StartLeftToScaleRight", Mode.BACKWARD_HIGH));
		 paths.put("StartLScaleL", new Path("StartLeftToScaleLeft", Mode.BACKWARD_HIGH));
		 paths.put("SwitchLScaleL", new Path("SwitchLeftToScaleLeft", Mode.BACKWARD_HIGH));
		 paths.put("SwitchLScaleR", new Path("SwitchLeftToScaleRight", Mode.BACKWARD_HIGH));
		 paths.put("SwitchRScaleL", new Path("SwitchRightToScaleLeft", Mode.BACKWARD_HIGH));
		 paths.put("SwitchRScaleR", new Path("SwitchRightToScaleRight", Mode.BACKWARD_HIGH));
		 paths.put("ScaleLSwitchL", new Path("ScaleLeftToSwitchLeft", Mode.FORWARD_HIGH));
		 paths.put("ScaleLSwitchR", new Path("ScaleLeftToSwitchRight", Mode.FORWARD_HIGH));
		 paths.put("ScaleRSwitchL", new Path("ScaleRightToSwitchLeft", Mode.FORWARD_HIGH));
		 paths.put("ScaleRSwitchR", new Path("ScaleRightToSwitchRight", Mode.FORWARD_HIGH));
		 paths.put("Turn" , new Path("Turn", Mode.FORWARD_HIGH));
		 paths.put("Drive straight", new Path("MotionProfileOne", Mode.BACKWARD_HIGH));
		 paths.put("StartLScaleL2", new Path("StartLeftToScaleLeft2Cube", Mode.BACKWARD_HIGH));
		 paths.put("StartRScaleR2", new Path("StartRightToScaleRight2Cube", Mode.BACKWARD_HIGH));
		 paths.put("StartRScaleL2", new Path("StartRightToScaleLeft2Cube", Mode.BACKWARD_HIGH));
		 paths.put("StartLScaleR2", new Path("StartLeftToScaleRight2Cube", Mode.BACKWARD_HIGH));
		 paths.put("StartRScaleLD", new Path("StartRightToScaleLeftDrive", Mode.BACKWARD_HIGH));
		 paths.put("StartLScaleRD", new Path("StartLeftToScaleRightDrive", Mode.BACKWARD_HIGH));
		 paths.put("ScaleNull", new Path("ScaleNullZone", Mode.BACKWARD_HIGH));
	 };
	 public static Path getPath(String path) {
		 return paths.get(path);
	 }
	
}
