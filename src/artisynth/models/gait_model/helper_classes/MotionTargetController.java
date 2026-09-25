package artisynth.models.gait_model.helper_classes;

import artisynth.core.inverse.TrackingController;
import artisynth.core.mechmodels.ExcitationComponent;
import artisynth.core.mechmodels.MechModel;

/**
 * The motion target controller is a subclass of {@link TrackingController} with
 * additional functionalities for tracking <em>excitation</em> or <em>force</em>
 * targets on pure-excitation actuators (FrameExciter, PointExciter,
 * JointActuator) and muscles via the embedded
 * {@link ForceExcitationTargetTerm}. Also the motion target controller uses
 * {@link FeasibleQPSolver} per default.
 *
 * @author Alexander Graf Copyright (c) 2026
 * <p>
 * University of Duisburg-Essen
 * <p>
 * Chair of Mechanics and Robotics
 * <p>
 * alexander.graf-lmr@uni-due.de
 */
public class MotionTargetController extends TrackingController {

   /**
    * The single cost term that handles all excitation/force targets on
    * pure-excitation actuators and muscles. Allocated lazily on first use.
    */
   protected ForceExcitationTargetTerm myForceExcitationTerm;

   /**
    * The installed solver, held separately from the inherited
    * {@code myQPSolver} field so its counters can be reached without a cast.
    */
   protected FeasibleQPSolver myFeasibleSolver;

   public MotionTargetController () {

   }

   public MotionTargetController (MechModel mech, String name) {
      super ();
      setMech (mech);
      setName (name);
      this.initComponents ();
      // Installed after initComponents, which is what allocates the stock
      // solver. See FeasibleQPSolver for why the stock one is not enough once
      // the motion targets are equality constraints.
      myFeasibleSolver = new FeasibleQPSolver (this);
      myQPSolver = myFeasibleSolver;
   }

   public void initialize (double t0) {
      super.initialize (t0);
      if (myFeasibleSolver != null) {
         myFeasibleSolver.reset ();
      }
      System.out
         .println ("MotionTargetController: run " + this.getMech ().getName ());
   }

   public void apply (double t0, double t1) {
      super.apply (t0, t1);
      System.out.println ("t = " + t1);
   }

   /**
    * Ensures that {@link #myForceExcitationTerm} exists and is registered as
    * a child cost term of this controller. Called lazily by every
    * {@code add...Target} method below.
    */
   protected ForceExcitationTargetTerm ensureForceExcitationTerm() {
      if (myForceExcitationTerm == null) {
         myForceExcitationTerm = new ForceExcitationTargetTerm (this);
         myForceExcitationTerm.setName ("forceExcitationTerm");
         addCostTerm (myForceExcitationTerm);
      }
      return myForceExcitationTerm;
   }

   /**
    * Returns the embedded {@link ForceExcitationTargetTerm}, allocating it
    * if necessary so callers can configure properties directly.
    */
   public ForceExcitationTargetTerm getForceExcitationTargetTerm() {
      return ensureForceExcitationTerm();
   }

   /**
    * Sets the overall weight of the {@link ForceExcitationTargetTerm}.
    * Per-target weights are passed in at registration via
    * {@link #addExcitationTarget} and {@link #addForceTarget}.
    */
   public void setForceExcitationTargetTermWeight (double w) {
      ensureForceExcitationTerm().setWeight (w);
   }

   public double getForceExcitationTargetTermWeight() {
      if (myForceExcitationTerm == null) {
         return ForceExcitationTargetTerm.DEFAULT_WEIGHT;
      }
      return myForceExcitationTerm.getWeight();
   }

   /**
    * Adds an excitation target for {@code ex}. The component must already
    * be on this controller's exciter list (added via {@link #addExciter}).
    *
    * @param ex excitation component to pin
    * @param targetExcitation desired excitation value
    * @param weight per-target QP weight
    */
   public void addExcitationTarget (
      ExcitationComponent ex, double targetExcitation, double weight) {
      ensureForceExcitationTerm()
         .addExcitationTarget (ex, targetExcitation, weight);
   }

   /**
    * Adds an excitation target with unit per-target weight.
    */
   public void addExcitationTarget (
      ExcitationComponent ex, double targetExcitation) {
      addExcitationTarget (ex, targetExcitation, 1.0);
   }

   /**
    * Adds a force target for {@code comp}. Supported types are
    * FrameExciter, PointExciter, JointActuator, and Muscle. See
    * {@link ForceExcitationTargetTerm#addForceTarget} for details.
    *
    * @param comp component to track
    * @param targetForce desired force (or generalized force / moment)
    * @param weight per-target QP weight
    */
   public void addForceTarget (
      ExcitationComponent comp, double targetForce, double weight) {
      ensureForceExcitationTerm().addForceTarget (comp, targetForce, weight);
   }

   /**
    * Adds a force target with unit per-target weight.
    */
   public void addForceTarget (ExcitationComponent comp, double targetForce) {
      addForceTarget (comp, targetForce, 1.0);
   }

   /**
    * Removes all excitation and force targets registered with the embedded
    * {@link ForceExcitationTargetTerm}. The term itself remains attached.
    */
   public void clearExcitationAndForceTargets() {
      if (myForceExcitationTerm != null) {
         myForceExcitationTerm.clearTargets();
      }
   }

   /**
    * The solver that checks each solution against the program it came from.
    * See {@link FeasibleQPSolver}.
    */
   public FeasibleQPSolver getFeasibleSolver () {
      return myFeasibleSolver;
   }

   /**
    * Share of this run's solves that had to be re-run without their equality
    * constraints. A run with a few percent here tracked its targets as
    * constraints; one with most of them did not, and the two produce output
    * that looks alike.
    *
    * @return fallback share in percent
    */
   public double getFallbackPercent () {
      return myFeasibleSolver == null ? 0 : myFeasibleSolver.getFallbackPercent ();
   }

   /**
    * @return number of solves re-run without their equality constraints
    */
   public int getFallbackCount () {
      return myFeasibleSolver == null ? 0 : myFeasibleSolver.getFallbackCount ();
   }

   /**
    * Returns the number of currently registered excitation/force targets.
    */
   public int numExcitationAndForceTargets() {
      if (myForceExcitationTerm == null) {
         return 0;
      }
      return myForceExcitationTerm.numTargets();
   }
}
