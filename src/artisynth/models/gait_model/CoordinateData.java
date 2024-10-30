package artisynth.models.gait_model;

import java.util.ArrayList;
import java.util.Collection;

public class CoordinateData {
   protected ArrayList<ArrayList<Double>> myCoordData = new ArrayList<> ();
   protected ArrayList<String> myCoordLabels = new ArrayList<String> ();
   protected ArrayList<Double> myFrameTimes = new ArrayList<Double> ();
   protected int myColumns;

   public CoordinateData () {
   }

   public void addData (double time, Collection<Double> coords) {
      if (numLabels () == 0) {
         throw new IllegalArgumentException ("No coordinate labels defined.");
      }
      myFrameTimes.add (time);
      ArrayList<Double> crds = new ArrayList<Double> ();
      coords.forEach (c -> {
         crds.add (c);
      });
      myCoordData.add (crds);
   }

   public ArrayList<String> getCoordLabels () {
      return myCoordLabels;
   }

   public ArrayList<Double> getData (int frame) {
      ArrayList<Double> coords = myCoordData.get (frame);
      return coords;
   }

   public Double getData (int frame, int idx) {
      Double coords = myCoordData.get (frame).get (idx);
      return coords;
   }

   public Double getData (int frame, String name)
      throws IllegalArgumentException {
      if (getCoordLabels ().contains (name)) {
         int num = getCoordLabels ().indexOf (name);
         Double coords = this.getData (frame, num);
         return coords;
      }
      else {
         throw new IllegalArgumentException (name + " is unknown.");
      }
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

   public int numColumns () {
      return myColumns;
   }

   public int numLabels () {
      return myCoordLabels.size ();
   }

   public int numFrames () {
      return myFrameTimes.size ();
   }

   public void setColumns (int size) {
      myColumns = size;
   }

   public void setCoordLabels (ArrayList<String> labels) {
      myCoordLabels = labels;
   }
}