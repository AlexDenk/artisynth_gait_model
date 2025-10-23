package artisynth.models.gait_model;

import java.util.ArrayList;
import java.util.Deque;

import artisynth.core.mechmodels.*;
import artisynth.core.modelbase.*;
import maspack.matrix.Matrix;
import maspack.matrix.SparseNumberedBlockMatrix;
import maspack.properties.PropertyList;
import maspack.properties.PropertyMode;
import maspack.properties.PropertyUtils;
import maspack.util.DataBuffer;
import maspack.util.DoubleInterval;

/**
 * @author Alexander Denk Copyright (c) 2025 <br>
 * (UDE) University of Duisburg-Essen <br>
 * Chair of Mechanics and Robotics <br>
 * alexander.denk@uni-due.de
 */
public class CoordinateActuator extends ModelComponentBase
implements ExcitationComponent, ForceComponent, HasNumericState {
   protected String myCoordinate;
   protected JointBase myJoint;
   protected double myMaxForce;
   protected double myExcitation;
   protected DoubleInterval myExcitationBounds = new DoubleInterval();
   protected PropertyMode myExcitationBoundsMode = PropertyMode.Inherited;
   protected double myTmp;

   protected CombinationRule myComboRule = CombinationRule.Sum;
   protected ExcitationSourceList myExcitationSources;

   /**
    * Creates a CoordinateActuator
    * 
    * @param name
    * name of the exciter, or {@code null}
    * @param joint
    * joint that the coordinate is referring to
    * @param coordinate
    * joint coordinate being activated
    * @param maxForce
    * maximum force along the coordinate
    */
   public CoordinateActuator (String name, JointBase joint, String coordinate,
   double maxForce) {
      super (name);
      myJoint = joint;
      myMaxForce = maxForce;
      this.setExcitationBounds (new DoubleInterval (-1, 1));
      // Do we need to check, if the coordinate actually belongs to the joint?
      boolean match = false;
      for (int i = 0; i < myJoint.numCoordinates (); i++) {
         String coord = myJoint.getCoordinateName (i);
         if (coordinate == coord) {
            match = true;
         }
      }
      if (match == false) {
         System.out
            .println (
               "Warning: could not create CoordinateActuator, because joint "
               + coordinate + " is not contained in " + joint.getName ());
         myCoordinate = null;
      }
      else {
         myCoordinate = coordinate;
      }
   }

   public CoordinateActuator (JointBase joint, String coordinate,
   double maxForce) {
      this (null, joint, coordinate, maxForce);
   }

   public CoordinateActuator (String name) {
      super (name);
   }

   public CoordinateActuator () {
      this (null);
   }

   public static PropertyList myProps =
      new PropertyList (CoordinateActuator.class, ModelComponentBase.class);
   
   static {
      myProps.add ("coordinate * *", "name of the joint coordinate for this exciter", null);
      myProps.add ("maxForce * *", "external force magnitude for excitation=1", null);
      myProps.add ("excitation * *", "excitation for the external force", null);
   }
   
   public PropertyList getAllPropertyInfo () {
      return myProps; 
   }

   @Override
   public void applyForces (double t) {
      myTmp = myExcitation * myMaxForce;
      int idx = myJoint.getCoordinateIndex (myCoordinate);
      myJoint.applyCoordinateForce (idx, myTmp);
   }

   @Override
   public void addSolveBlocks (SparseNumberedBlockMatrix M) {
   }

   @Override
   public void addPosJacobian (SparseNumberedBlockMatrix M, double s) {
   }

   @Override
   public void addVelJacobian (SparseNumberedBlockMatrix M, double s) {
   }

   @Override
   public int getJacobianType () {
      return Matrix.SPD;
   }

   @Override
   public void getState (DataBuffer data) {
      data.dput (myExcitation);
   }

   @Override
   public void setState (DataBuffer data) {
      myExcitation = data.dget();
   }

   @Override
   public void setCombinationRule (CombinationRule rule) {
      myComboRule = rule;
   }
   
   public boolean hasState() {
      return true;
   }

   @Override
   public CombinationRule getCombinationRule () {
      return myComboRule;
   }

   @Override
   public void setExcitation (double e) {
      myExcitation = e;
   }

   @Override
   public double getExcitation () {
      return myExcitation;
   }

   public void setExcitationBounds (DoubleInterval bounds) {
      myExcitationBounds.set (bounds);
      myExcitationBoundsMode = PropertyUtils.propagateValue (
               this, "excitationBounds", bounds, myExcitationBoundsMode);
   }

   public void setExcitationBounds (double lower, double upper) {
      this.setExcitationBounds (new DoubleInterval (lower, upper));
   }

   public DoubleInterval getExcitationBounds () {
      return myExcitationBounds;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public void addExcitationSource (ExcitationComponent ex, double gain) {
      if (myExcitationSources == null) {
         myExcitationSources = new ExcitationSourceList ();
      }
      myExcitationSources.add (ex, gain);
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public boolean setExcitationGain (ExcitationComponent ex, double gain) {
      return ExcitationUtils.setGain (myExcitationSources, ex, gain);
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public double getExcitationGain (ExcitationComponent ex) {
      return ExcitationUtils.getGain (myExcitationSources, ex);
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public boolean removeExcitationSource (ExcitationComponent ex) {
      boolean removed = false;
      if (myExcitationSources != null) {
         removed = myExcitationSources.remove (ex);
         if (myExcitationSources.size () == 0) {
            myExcitationSources = null;
         }
      }
      return removed;
   }

   @Override
   public double getNetExcitation () {
      return ExcitationUtils.combine (
         myExcitation, myExcitationSources, myComboRule);
   }

   public void setMaxForce (double maxForce) {
      myMaxForce = maxForce;
   }

   public double getMaxForce () {
      return myMaxForce;
   }

   public void setCoordinate (String coordinate) {
      myCoordinate = coordinate;
   }

   public String getCoordinate () {
      return myCoordinate;
   }
   
   /**
    * {@inheritDoc}
    */
   @Override
   public void updateReferences (boolean undo, Deque<Object> undoInfo) {
      super.updateReferences (undo, undoInfo);
      myExcitationSources = ExcitationUtils.updateReferences (
         this, myExcitationSources, undo, undoInfo);
   }

   /**
    * Creates a complete set of CoordinateActuators for all coordinates, that
    * are associated with that joint.
    * 
    * @param mech
    * MechModel to add the exciters to
    * @param joint
    * joint for which the exciters should be created
    * @param maxForce
    * maximum moment about any axis
    * @return list of the created exciters
    */
   public static ArrayList<CoordinateActuator> createCoordinateActuators (
      MechModel mech, JointBase joint, double maxForce) {
      ArrayList<CoordinateActuator> exs = new ArrayList<> ();
      for (int i = 0; i < joint.numCoordinates (); i++) {
         String coordinate = joint.getCoordinateName (i);
         String name = coordinate + "_reserve";
         exs.add (new CoordinateActuator (name, joint, coordinate, maxForce));
      }
      if (mech != null) {
         for (CoordinateActuator ex : exs) {
            mech.addForceEffector (ex);
         }
      }
      return exs;
   }
}