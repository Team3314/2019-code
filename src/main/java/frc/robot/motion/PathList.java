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
	 };
	 public static Path getPath(String path) {
		 return paths.get(path);
	 }
	
}
