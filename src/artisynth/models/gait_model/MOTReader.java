package artisynth.models.gait_model;

import java.io.*;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

import maspack.matrix.*;

/**
 * File importer for .mot files.
 * <p>
 * 
 * @author Alexander Denk Copyright (c) 2025
 * <p>
 * University of Duisburg-Essen
 * <p>
 * Chair of Mechanics and Robotics
 * <p>
 * alexander.denk@uni-due.de
 */
public class MOTReader {
   // ----------------------------Instance Fields------------------------------
   protected InputStream myIstream;
   protected File myFile;
   protected ForceData myForces = new ForceData ();
   protected CoordinateData myCoords = new CoordinateData ();
   protected boolean coordsInDegrees;
   protected boolean forcesInDegrees;
   protected String myP1ContactSide;
   protected int myP1ContactPlate;

   // ------------------------------Constructors-------------------------------
   public MOTReader () {
   }

   public MOTReader (InputStream is) {
      myIstream = is;
   }

   public MOTReader (InputStream is, String side, int num) {
      myIstream = is;
      myP1ContactSide = side.toLowerCase ();
      myP1ContactPlate = num;
   }

   public MOTReader (File file, String side, int num) throws IOException {
      this (new FileInputStream (file), side, num);
      myFile = file;
   }

   public MOTReader (File file) throws IOException {
      this (new FileInputStream (file));
   }

   // --------------------------Static Methods---------------------------------
   public static void main (String[] args) {

   }

   // -------------------------Instance Methods--------------------------------
   public void close () {
      closeQuietly (myIstream);
   }

   public CoordinateData getCoordinateData () {
      return myCoords;
   }

   public ForceData getForceData () {
      return myForces;
   }

   public int getNumCoordFrames () {
      return myCoords.numFrames ();
   }

   public int getNumForceFrames () {
      return myForces.numFrames ();
   }

   public int getNumCoordLabels () {
      return myCoords.numLabels ();
   }

   public int getNumForceLabels () {
      return myForces.numLabels ();
   }

   public boolean getCoordsInDegrees () {
      return coordsInDegrees;
   }

   public boolean getForcesInDegrees () {
      return forcesInDegrees;
   }

   public List<Object> scanDataHeader (BufferedReader reader)
      throws IOException {
      int numLines = 0;
      int numColumns = 0;
      boolean inDegrees = false;
      String line;
      // Read general info
      do {
         if ((line = reader.readLine ()) != null) {
            line = line.trim ();
            if (line.contains ("nColumns")) {
               String[] tokens = line.split ("=");
               numColumns = Integer.parseInt (tokens[1]);
               numLines++;
               continue;
            }
            else if (line.contains ("inDegrees")) {
               String[] tokens = line.split ("=");
               if (tokens.length >= 2 && tokens[1].equals ("yes")) {
                  inDegrees = true;
               }
               numLines++;
               continue;
            }
            numLines++;
         }
      }
      while (!line.contains ("endheader"));
      // Read labels
      boolean forcefile = false;
      if ((line = reader.readLine ()) != null) {
         line = line.trim ();
         String[] tokens = line.split ("\t");
         for (int i = 0; i < tokens.length; i++) {
            if (tokens[i].contains ("force") || tokens[i].contains ("torque")) {
               forcefile = true;
               break;
            }
         }
         if (forcefile) {
            myForces.setForceLabels (readForceLabels (tokens));
            myForces.setColumns (numColumns);
            forcesInDegrees = inDegrees;
         }
         else {
            myCoords.setCoordLabels (readCoordLabels (tokens));
            myCoords.setColumns (numColumns);
            coordsInDegrees = inDegrees;
         }
         numLines++;
      }
      // Return several flags for the data scanner
      ArrayList<Object> flags = new ArrayList<Object> ();
      flags.add (numLines);
      flags.add (numColumns);
      flags.add (forcefile);
      return flags;
   }

   public void scanMOTData (BufferedReader reader, List<Object> flags)
      throws IOException {
      int numLines = (int)flags.get (0);
      int numColumns = (int)flags.get (1);
      boolean forcefile = (boolean)flags.get (2);
      String line;
      while ((line = reader.readLine ()) != null) {
         line = line.trim ();
         if (line.length () != 0) {
            String[] data = line.split ("\t");
            double time = Double.parseDouble (data[0]);
            if (data.length != numColumns) {
               throw new IOException (
                  "Line " + numLines + ": detected " + data.length
                  + " fields; expected " + numColumns);
            }
            // Read all data entries
            if (forcefile) {
               List<Vector3d> forces = new ArrayList<Vector3d> ();
               for (int i = 1; i < data.length; i++) {
                  Vector3d force = new Vector3d ();
                  force.x = Double.parseDouble (data[i]);
                  force.y = Double.parseDouble (data[++i]);
                  force.z = Double.parseDouble (data[++i]);
                  forces.add (force);
               }
               myForces.addData (time, forces);
            }
            else {
               List<Double> coords = new ArrayList<Double> ();
               for (int i = 1; i < data.length; i++) {
                  coords.add (Double.parseDouble (data[i]));
               }
               myCoords.addData (time, coords);
            }
         }
         numLines++;
      }
   }

   public void readData () throws IOException {
      BufferedReader reader =
         new BufferedReader (new InputStreamReader (myIstream));
      // How many lines, columns and which kind of file?
      List<Object> flags = scanDataHeader (reader);
      scanMOTData (reader, flags);
      reader.close ();
   }

   private void closeQuietly (InputStream in) {
      if (in != null) {
         try {
            in.close ();
         }
         catch (IOException e) {
         }
      }
   }

   private ArrayList<String> readCoordLabels (String[] tokens) {
      ArrayList<String> labels = new ArrayList<String> ();
      // read labels as they are, but ignore time
      for (int i = 1; i < tokens.length; i++) {
         labels.add (tokens[i]);
      }
      return labels;
   }

   private ArrayList<String> readForceLabels (String[] tokens) {
      Set<String> labels = new LinkedHashSet<String> ();
      // Assign plates 1 and 2 to left or right side each
      String p1 =
         (myP1ContactSide.equals ("right") && myP1ContactPlate == 1)
         || (myP1ContactSide.equals ("left") && myP1ContactPlate == 2) ? "Right"
            : "Left";
      String p2 = p1.equals ("Right") ? "Left" : "Right";
      // Combine each column header triple (x,y,z) to a single
      // label, skip time entry so start at 1.
      for (int i = 1; i < tokens.length; i++) {
         String plate = tokens[i].contains ("1") ? p1 : p2;
         if (tokens[i].contains ("v")) {
            labels.add (String.format ("%s GRF", plate));
         }
         else if (tokens[i].contains ("p")) {
            labels.add (String.format ("%s COP", plate));
         }
         else if (tokens[i].contains ("torque")
         || tokens[i].contains ("moment")) {
            labels.add (String.format ("%s GRM", plate));
         }
      }
      return new ArrayList<String> (labels);
   }
}