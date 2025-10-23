package artisynth.models.gait_model;

import java.io.*;
import java.util.ArrayList;
import java.util.List;

import artisynth.core.probes.TRCReader;
import maspack.matrix.Vector3d;
import maspack.util.ReaderTokenizer;

/**
 * @author Alexander Denk Copyright (c) 2023-2024 <br>
 * (UDE) University of Duisburg-Essen <br>
 * Chair of Mechanics and Robotics <br>
 * alexander.denk@uni-due.de
 */
public class CustomTRCReader extends TRCReader {
   // ----------------------------Instance Fields------------------------------
   protected MarkerMapping myMap;

   // ------------------------------Constructors-------------------------------
   public CustomTRCReader (File file, MarkerMapping map) throws IOException {
      super (file);
      myMap = map;
   }

   // --------------------------Static Methods---------------------------------
   public static void main (String[] args) {

   }

   // -------------------------Instance Methods--------------------------------
   
   public String getUnits() {
      return myUnits;
   }

   
   @Override
   public void readData () throws IOException {
      BufferedReader reader =
         new BufferedReader (new InputStreamReader (myIstream));
      scanFileHeader (reader);
      ReaderTokenizer rtok = new ReaderTokenizer (reader);
      rtok.eolIsSignificant (true);
      List<Object> flags = scanDataHead (reader);
      scanPositionData (reader, flags);
   }

   protected List<Object> scanDataHead (BufferedReader reader)
      throws IOException {
      // Start with offset 1 since data header is already read at this point
      int numLines = 1;
      String line = reader.readLine ();
      String[] labels = line.split ("\t");
      numLines++;
      line = reader.readLine ();
      String[] values = line.split ("\t");
      numLines++;
      if (labels.length != values.length) {
         throw new IOException (
            "Number of labels on line 2 differs from number of values on line 3");
      }
      int numMarkers = 0;
      int numFrames = 0;
      for (int i = 0; i < labels.length; i++) {
         String label = labels[i].trim ();
         if (label.equalsIgnoreCase ("DataRate")) {
            myDataRate = scanDouble (values[i], 3, i + 1);
         }
         else if (label.equalsIgnoreCase ("CameraRate")) {
            myCameraRate = scanDouble (values[i], 3, i + 1);
         }
         else if (label.equalsIgnoreCase ("NumFrames")) {
            numFrames = scanInt (values[i], 3, i + 1);
         }
         else if (label.equalsIgnoreCase ("NumMarkers")) {
            numMarkers = scanInt (values[i], 3, i + 1);
         }
         else if (label.equalsIgnoreCase ("Units")) {
            myUnits = values[i].trim ();
         }
         else if (label.equalsIgnoreCase ("OrigDataRate")) {
            myOrigDataRate = scanDouble (values[i], 3, i + 1);
         }
         else if (label.equalsIgnoreCase ("OrigDataStartFrame")) {
            myOrigDataStartFrame = scanInt (values[i], 3, i + 1);
         }
         else if (label.equalsIgnoreCase ("OrigNumFrames")) {
            myOrigNumFrames = scanInt (values[i], 3, i + 1);
         }
         else {
            System.out
               .println (
                  "Warning: TRCReader: unrecognized data label '" + label
                  + "', ignoring");
         }
      }
      line = reader.readLine ();
      labels = line.split ("\t");
      numLines++;
      ArrayList<String> expLabels = myMap.getExpLabels ();
      ArrayList<String> mkrLabels = new ArrayList<String> ();
      ArrayList<Integer> idxs = new ArrayList<Integer> ();
   // ignore first two labels time and frame
      for (int i = 2; i < labels.length; i += 3) {
         String label = labels[i].trim ();
         if (label.length () == 0) {
            throw new IOException (
               "Line 4: label for marker" + (i + 1) + " is empty");
         }
         // Filter marker labels, that are not supposed to be added
         if (expLabels.contains (label)) {
            mkrLabels.add (label);
            idxs.add (i);
         }
         else
            continue;
      }
      if (mkrLabels.size () != numMarkers) {
         System.out
            .println (
               "Warning: TRCReader: num marker labels (" + mkrLabels.size ()
               + ") != specified number of markers (" + numMarkers
               + "), assuming " + mkrLabels.size ());
      }
      myMotionData.setMarkerLabels (mkrLabels);
      // read all lines until the data start
      do {
         line = reader.readLine ();
         numLines++;
      }
      while(!line.equals(""));
      ArrayList<Object> flags = new ArrayList<Object> ();
      flags.add (numFrames);
      flags.add (numLines);
      flags.add (idxs);
      return flags;
   }

   @SuppressWarnings("unchecked")
   protected void scanPositionData (BufferedReader reader, List<Object> flags)
      throws IOException {
      String line;
      int numMarkers = myMotionData.numMarkers ();
      int numFrames = (int)flags.get (0);
      int lineno = (int)flags.get (1);
      ArrayList<Integer> idxs = (ArrayList<Integer>)flags.get (2);

      while ((line = reader.readLine ()) != null) {
         line = line.trim ();
         if (line.length () != 0) {
            String[] fields = line.split ("\t");
            ArrayList<Vector3d> positions = new ArrayList<> ();
            if (fields.length < 2 + numMarkers * 3) {
               throw new IOException (
                  "Line " + lineno + ": detected " + fields.length
                  + " fields; expected minimum " + (2 + numMarkers * 3));
            }
            double time = scanDouble (fields[1], lineno, 2);
            for (int j = 0; j < numMarkers; j++) {
               Vector3d pos = new Vector3d ();
               pos.x = scanDouble (fields[idxs.get (j)], lineno, idxs.get (j));
               pos.y =
                  scanDouble (
                     fields[idxs.get (j) + 1], lineno, idxs.get (j) + 1);
               pos.z =
                  scanDouble (
                     fields[idxs.get (j) + 2], lineno, idxs.get (j) + 2);
               positions.add (pos);
            }
            myMotionData.addFrame (time, positions);
         }
         lineno++;
      }
      if (myMotionData.numFrames () != numFrames) {
         System.out
            .println (
               "Warning: TRCReader: reader " + myMotionData.numFrames ()
               + " frames; expected " + numFrames);
      }
   }
}