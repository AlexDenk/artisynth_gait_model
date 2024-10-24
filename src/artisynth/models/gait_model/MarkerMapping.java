package artisynth.models.diss;

import java.util.ArrayList;

/**
 * Data structure that contains the names of experimental markers and virtual
 * markers alongside their corresponding weights.
 * 
 * @author Alexander Denk Copyright (c) 2024, by the author: Alexander Denk
 * (UDE) University of Duisburg-Essen Chair of Mechanics and Robotics
 * alexander.denk@uni-due.de
 */
public class MarkerMapping {
   // ----------------------------Instance Fields------------------------------
   protected ArrayList<String> myModelLabels;
   protected ArrayList<String> myExpLabels;
   protected ArrayList<Double> myWeights;

   // -------------------------------Constructors------------------------------
   public MarkerMapping () {
   }

   public MarkerMapping (ArrayList<String> model, ArrayList<String> exp,
   ArrayList<Double> wts) {
      assert model.size () == exp
         .size () : "Error: Dimension mismatch between marker and exp labels!";
      assert exp.size () == wts
         .size () : "Error: Dimension mismatch between exp labels and number of weights!";
      assert model.size () == wts
         .size () : "Error: Dimension mismatch between marker labels and number of weights!";

      setModelLabels (model);
      setExpLabels (exp);
      setWeights (wts);
   }

   // -------------------------------Instance Methods--------------------------
   /**
    * Returns the experimental marker label corresponding to the given model
    * marker label.
    * 
    * @param modelLabel
    * model marker label
    * @return String experimental marker label or null if the given label is
    * unknown
    */
   public String getExpLabelFromModel (String modelLabel) {
      if (myModelLabels.contains (modelLabel)) {
         int index = myModelLabels.indexOf (modelLabel);
         return myExpLabels.get (index);
      }
      else {
         return null;
      }
   }

   /**
    * Returns a list of all experimental marker labels.
    * 
    * @return {@code ArrayList<String>} experimental marker labels
    */
   public ArrayList<String> getExpLabels () {
      return myExpLabels;
   }

   /**
    * Returns the label at index {@code num} from the list of experimental
    * labels.
    * 
    * @param num
    * index
    * @return String experimental marker label
    */
   public String getExpLabel (int num) {
      return myExpLabels.get (num);
   }

   /**
    * Returns a list of all experimental marker weights.
    * 
    * @return {@code ArrayList<Double>} marker weights
    */
   public ArrayList<Double> getMarkerWeights () {
      return myWeights;
   }

   /**
    * Returns the weight at index {@code num} from the list of marker weights.
    * 
    * @param num
    * index
    * @return Double weight
    */
   public Double getMarkerWeight (int num) {
      return myWeights.get (num);
   }

   /**
    * Returns the weight corresponding to the given experimental or model marker
    * label.
    * 
    * @param label
    * String label
    * @return Double weight or null if the given label is unknown
    */
   public Double getMarkerWeight (String label) {
      int index;
      if (myModelLabels.contains (label)) {
         index = myModelLabels.indexOf (label);
      }
      else if (myExpLabels.contains (label)) {
         index = myExpLabels.indexOf (label);
      }
      else {
         return 1.0;
      }
      return myWeights.get (index);
   }

   /**
    * Returns the model marker label corresponding to the given experimental
    * marker label.
    * 
    * @param expLabel
    * experimental marker label
    * @return String model marker label or null if the given label is unknown
    */
   public String getModelLabelFromExp (String expLabel) {
      if (myExpLabels.contains (expLabel)) {
         int index = myExpLabels.indexOf (expLabel);
         return myModelLabels.get (index);
      }
      else {
         return null;
      }
   }

   /**
    * Returns a list of all model marker labels.
    * 
    * @return {@code ArrayList<String>} model marker labels
    */
   public ArrayList<String> getModelLabels () {
      return myModelLabels;
   }

   /**
    * Returns the model marker label at index {@code num} from the list of model
    * marker names.
    * 
    * @param num
    * index
    * @return String model marker label
    */
   public String getModelLabel (int num) {
      return myModelLabels.get (num);
   }

   private void setModelLabels (ArrayList<String> labels) {
      myModelLabels = labels;
   }

   private void setExpLabels (ArrayList<String> labels) {
      myExpLabels = labels;
   }

   private void setWeights (ArrayList<Double> weights) {
      myWeights = weights;
   }
}