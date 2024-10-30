package artisynth.models.gait_model;

import java.util.ArrayList;
import java.util.Collection;
import maspack.matrix.Vector3d;

public class ForceData {
      protected ArrayList<ArrayList<Vector3d>> myForceData = new ArrayList<> ();
      protected ArrayList<String> myForceLabels = new ArrayList<String> ();
      protected ArrayList<Double> myFrameTimes = new ArrayList<Double> ();
      protected int myColumns;

      public ForceData () {
      }

      public void addData (double time, Collection<Vector3d> forces) {
         if (numLabels () == 0) {
            throw new IllegalArgumentException ("No force labels defined.");
         }
         myFrameTimes.add (time);
         ArrayList<Vector3d> frcs = new ArrayList<Vector3d> ();
         forces.forEach (f -> {
            frcs.add (f);
         });
         myForceData.add (frcs);
      }

      public ArrayList<Vector3d> getData (int frame) {
         ArrayList<Vector3d> forces = myForceData.get (frame);
         return forces;
      }

      public Vector3d getData (int frame, int idx) {
         Vector3d force = myForceData.get (frame).get (idx);
         return force;
      }

      public Vector3d getData (int frame, String name)
         throws IllegalArgumentException {
         if (getForceLabels ().contains (name)) {
            int num = getForceLabels ().indexOf (name);
            Vector3d force = this.getData (frame, num);
            return force;
         }
         else {
            throw new IllegalArgumentException (name + " is unknown.");
         }
      }

      public ArrayList<String> getForceLabels () {
         return myForceLabels;
      }

      public int getFrame (double t1) {
         if (t1 > myFrameTimes.get (numFrames () - 1)) {
            throw new IllegalArgumentException ("Time t1 out of range.");
         }
         int frame = 0;
         for (Double time : myFrameTimes) {
            if (time >= t1) {
               frame = myFrameTimes.indexOf (time);
               break;
            }
         }
         return frame;
      }

      public double getFrameTime (int frame) {
         return myFrameTimes.get (frame);
      }

      public double getMaxForce (String label) {
         int num = getForceLabels ().indexOf (label);
         ArrayList<Vector3d> forces = new ArrayList<Vector3d> ();
         myFrameTimes.forEach (t -> {
            Vector3d force = this.getData (this.getFrame (t), num);
            forces.add (force);
         });
         double max = Double.MIN_VALUE;
         for (Vector3d f : forces) {
            if (f.maxElement () > max) {
               max = f.maxElement ();
            }
         }
         return max;
      }

      public double getMaxMoment (String label) {
         int num = getForceLabels ().indexOf (label);
         ArrayList<Vector3d> moments = new ArrayList<Vector3d> ();
         myFrameTimes.forEach (t -> {
            Vector3d moment = this.getData (this.getFrame (t), num);
            moments.add (moment);
         });
         double max = Double.MIN_VALUE;
         for (Vector3d m : moments) {
            if (m.maxElement () > max) {
               max = m.maxElement ();
            }
         }
         return max;
      }

      public int numColumns () {
         return myColumns;
      }

      public int numLabels () {
         return myForceLabels.size ();
      }

      public int numFrames () {
         return myFrameTimes.size ();
      }

      public void setColumns (int size) {
         myColumns = size;
      }

      public void setForceLabels (ArrayList<String> labels) {
         myForceLabels = labels;
      }
   }