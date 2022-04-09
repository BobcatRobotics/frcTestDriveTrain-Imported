// /*
// This is a utility function made to convert pathweaver trajectories into manually-generated waypoint trajectories
// - The purpose of this is because pathweaver, thus far in 2022, does not allow you to apply voltage constraints to trajectories generated from their tool and functions
// - Currently, I could not figure out the JSON functionality in the function createPoseWaypoint, so I reverted it, leaving Jake's comments, and am cleaning up the code and attempting to get it functional, the way team 6391 wrote it
// - And yes, this was originally written by team 6391 and posted on an open Chief Delphi forum by team 1108 (credit: https://www.chiefdelphi.com/t/voltage-constraint-with-pathweaver-or-pathplanner/403145/4)

// Arnav
// */

// package frc.robot.utils;
// import java.io.FileReader;
// import java.nio.file.Path;
// import java.util.ArrayList;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
// import org.json.simple.JSONArray;
// import org.json.simple.JSONObject;
// import org.json.simple.parser.JSONParser;

// //team 6391 initially wrote this and posted this to an open repo chiefdelphi

// public class BobcatTrajectoryCreater {   
//    public static Trajectory fromWaypoints(Path path, TrajectoryConfig config) {
//       Pose2d first = null;
//       Pose2d last = null;
//       var waypoints = new ArrayList<Translation2d>();
//       JSONParser parser = new JSONParser();
//       try{
//          JSONArray jsonArray = (JSONArray) parser.parse(new FileReader(path.toString()));
//          Object[] jobArray = jsonArray.toArray();
         
//          JSONObject job =(JSONObject) jobArray[0];
//          first =(Pose2d) getPose(job, "pose");

//          job = (JSONObject) jobArray[jobArray.length-1];
//          last =(Pose2d) getPose(job, "pose");
         
//          //create all internal points
//          for (int i = 1; i < jobArray.length-1 ; i++){
//             job = (JSONObject) jobArray[i];
//             var point =(Translation2d) getPose(job,"trans");
//             // System.out.println(point);
//             waypoints.add(point);
//          }
//       } catch (Exception e) {

//       }
//       return TrajectoryGenerator.generateTrajectory(first, waypoints, last, config);
//    }

//    private static Object getPose(JSONObject job, String desire) {
//       JSONObject poseObject = (JSONObject) job.get("pose");
//       JSONObject translation = (JSONObject) poseObject.get("translation");
//       JSONObject rotation = (JSONObject) poseObject.get("rotation");

//       double x = (double) translation.get("x");
//       double y = (double) translation.get("y");
//       double r = (double) rotation.get("radians");

//       Translation2d trans = new Translation2d(x,y);
//       Rotation2d rot = new Rotation2d(r);

//       Pose2d pose = new Pose2d(trans, rot);

//       switch(desire){
//          case "pose":
//             return pose;
         
//          case "trans":
//             return trans;

//          default:
//             return null;
//       }
//    }
// }