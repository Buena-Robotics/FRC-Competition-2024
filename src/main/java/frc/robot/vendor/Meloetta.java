package frc.robot.vendor;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class Meloetta {
    public static class Collider2d{
        private Pose2d pose;
        private Translation2d size;
        public Collider2d(Pose2d pose, Translation2d size){
            this.pose = pose;
            this.size = size;
        }
        public Pose2d getPose(){ return pose; }
        public Translation2d getSize(){ return size; }
        public void setPose(Pose2d pose){ this.pose = pose; }
        public void setSize(Translation2d size){ this.size = size; }
        //https://github.com/ArthurGerbelot/rect-collide/blob/master/src/index.js
        //https://stackoverflow.com/questions/62028169/how-to-detect-when-rotated-rectangles-are-colliding-each-other
        public boolean isColidingWith(Collider2d other){ return true; }
    }
}
