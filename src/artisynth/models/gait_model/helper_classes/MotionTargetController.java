package artisynth.models.gait_model.helper_classes;

import artisynth.core.inverse.QPSolver;
import artisynth.core.femmodels.FemModel;
import artisynth.core.inverse.TrackingController;
import artisynth.core.mechmodels.ExcitationComponent;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MechSystemModel;
import artisynth.core.modelbase.ComponentList;
import artisynth.core.modelbase.ModelComponent;

/**
 * The motion target controller is a subclass of {@link TrackingController} with
 * additional functionalities for tracking <em>excitation</em> or <em>force</em>
 * targets on pure-excitation actuators (FrameExciter, PointExciter,
 * JointActuator) and muscles via the embedded
 * {@link ForceExcitationTargetTerm}.
 * <p>
 * It also computes the FEM stresses once per step for the excitation
 * response, instead of once per exciter. Before each force evaluation of the
 * response, the stock
 * {@link TrackingController#setExcitations(maspack.matrix.VectorNd, int)}
 * invalidates the stresses of every FEM model. Positions do not change during
 * the response, and only FEM muscle models have excitation-dependent
 * stresses, so this controller invalidates all FEM stresses once before the
 * response and, during it, only those of FEM models that are excitation
 * components.
 * <p>
 * The new functionality fills two gaps in the stock {@link TrackingController}:
 * <ol>
 * <li>Pure-excitation actuators ({@code F = e * scale}) have trivial
 * force/position Jacobians, so the inherited {@code ForceEffectorTerm} cannot
 * drive them toward a force target. The new term encodes their force target as
 * a direct excitation target.</li>
 * <li>Components driven by excitation alone, such as joint actuators, can now
 * be tracked by prescribing the excitation they should attain.</li>
 * </ol>
 * Because the new term is added to the same QP as the inherited motion and
 * force-effector terms, the solver finds excitations that simultaneously
 * realise the motion targets <em>and</em> the prescribed excitation/force
 * targets, with relative trade-off governed by the term weights.
 *
 * @author Alexander Denk Copyright (c) 2025
 * <p>
 * University of Duisburg-Essen
 * <p>
 * Chair of Mechanics and Robotics
 * <p>
 * alexander.denk@uni-due.de
 */
public class MotionTargetController extends TrackingController {

   /**
    * The single cost term that handles all excitation/force targets on
    * pure-excitation actuators and muscles. Allocated lazily on first use.
    */
   protected ForceExcitationTargetTerm myForceExcitationTerm;

   // True while the excitation response is computed
   private boolean myComputingResponse = false;

   public MotionTargetController () {

   }

   public MotionTargetController (MechModel mech, String name) {
      super ();
      setMech (mech);
      setName (name);
      this.initComponents ();
      // Configured after initComponents, which is what allocates the solver.
      // The retry is off by default in core; once the motion targets are
      // equality constraints a step the solver cannot satisfy would stall the
      // whole run rather than being approached as a cost. See QPSolver.
      myQPSolver.setFallbackEnabled (true);
   }

   public void initialize (double t0) {
      super.initialize (t0);
      if (myQPSolver != null) {
         myQPSolver.reset ();
      }
      System.out
         .println ("MotionTargetController: run " + this.getMech ().getName ());
   }

   public void apply (double t0, double t1) {
      super.apply (t0, t1);
      System.out.println ("t = " + t1);
   }

   /**
    * Computes the excitation response with all FEM stresses invalidated once
    * beforehand, so that its force evaluations use the stresses of the
    * current positions.
    *
    * @param t0 current time
    * @param t1 time at the end of the step
    */
   @Override
   protected void updateCostTerms (double t0, double t1) {
      super.invalidateFEMStresses (getMech ());
      myComputingResponse = true;
      try {
         super.updateCostTerms (t0, t1);
      }
      finally {
         myComputingResponse = false;
      }
   }

   /**
    * Invalidates the stresses of all FEM models or, while the excitation
    * response is computed, only those whose stresses depend on excitations,
    * i.e. FEM muscle models.
    *
    * @param model mech system whose FEM models are checked
    */
   @Override
   protected void invalidateFEMStresses (MechSystemModel model) {
      if (myComputingResponse) {
         invalidateExcitableFemStresses (model);
      }
      else {
         super.invalidateFEMStresses (model);
      }
   }

   // Same traversal as the stock method, which invalidates every FEM model
   private void invalidateExcitableFemStresses (ModelComponent comp) {
      if (comp instanceof FemModel) {
         if (comp instanceof ExcitationComponent) {
            ((FemModel)comp).invalidateStressAndStiffness ();
         }
      }
      else if (comp instanceof ComponentList<?>) {
         for (ModelComponent mc : (ComponentList<?>)comp) {
            invalidateExcitableFemStresses (mc);
         }
      }
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
    * See {@link QPSolver}.
    */
   public QPSolver getSolver () {
      return myQPSolver;
   }

   /**
    * @deprecated Use {@link #getSolver} instead. Kept because a caller that
    * probes for this method before reading the solver health would otherwise
    * find nothing and record none of it.
    *
    * @return the installed solver
    */
   @Deprecated
   public QPSolver getFeasibleSolver () {
      return myQPSolver;
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
      return myQPSolver == null ? 0 : myQPSolver.getFallbackPercent ();
   }

   /**
    * @return number of solves re-run without their equality constraints
    */
   public int getFallbackCount () {
      return myQPSolver == null ? 0 : myQPSolver.getFallbackCount ();
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
