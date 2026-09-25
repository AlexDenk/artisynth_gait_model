package artisynth.models.gait_model.helper_classes;

import java.util.ArrayList;

import artisynth.core.inverse.FrameExciter;
import artisynth.core.inverse.LeastSquaresTermBase;
import artisynth.core.inverse.PointExciter;
import artisynth.core.inverse.TrackingController;
import artisynth.core.mechmodels.ExcitationComponent;
import artisynth.core.mechmodels.ForceTargetComponent;
import artisynth.core.mechmodels.JointActuator;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MechSystemBase;
import artisynth.core.mechmodels.MechSystemSolver.Integrator;
import artisynth.core.mechmodels.Muscle;
import artisynth.core.modelbase.ComponentUtils;
import artisynth.core.util.TimeBase;
import maspack.matrix.MatrixNd;
import maspack.matrix.SparseBlockMatrix;
import maspack.matrix.VectorNd;

/**
 * A QP term that lets {@link MotionTargetController} (and any
 * {@link TrackingController}) track <em>excitation</em> or <em>force</em>
 * targets on active components.
 *
 * @author Alexander Graf Copyright (c) 2026
 * <p>
 * University of Duisburg-Essen
 * <p>
 * Chair of Mechanics and Robotics
 * <p>
 * alexander.graf-lmr@uni-due.de
 */
public class ForceExcitationTargetTerm extends LeastSquaresTermBase {

   public static final double DEFAULT_WEIGHT = 1.0;
   public static final double DEFAULT_FD_DELTA = 1e-4;

   /**
    * Kind of target represented internally. Named {@code TargetType} rather
    * than {@code Type} to avoid colliding with {@code QPTerm.Type}, which
    * {@link LeastSquaresTermBase} exposes through {@link #getType}.
    */
   public enum TargetType {
      /** Drive the component's excitation toward a prescribed value. */
      EXCITATION,
      /**
       * Drive the linear force {@code F = e * scale} (FrameExciter,
       * PointExciter, JointActuator) toward a prescribed value. Converted
       * exactly to an excitation target {@code e_target = F / scale}.
       */
      PURE_FORCE,
      /**
       * Drive a {@link Muscle}'s axial tension toward a prescribed value,
       * accounting for both activation and state coupling.
       */
      MUSCLE_FORCE
   }

   /** Internal record describing one target. */
   protected static class Target {
      ExcitationComponent comp;
      TargetType type;
      double targetValue;        // excitation value or force, per type
      double weight;             // per-target QP weight
      int cachedIndex = -1;      // index into the controller's exciter list
      double inverseScale = 1.0; // 1/maxForce or 1/forceScale (PURE_FORCE)
      boolean warnedUnreachable; // so the warning is printed only once
   }

   protected ArrayList<Target> myTargets = new ArrayList<Target>();
   protected TrackingController myController;
   protected double myFdDelta = DEFAULT_FD_DELTA;

   protected MatrixNd myH = new MatrixNd();
   protected VectorNd myB = new VectorNd();

   // scratch, reused between steps
   protected VectorNd myCurVel = new VectorNd();
   protected VectorNd myF0 = new VectorNd(1);
   protected VectorNd myMinf = new VectorNd(1);
   protected VectorNd myHfj = new VectorNd(1);

   public ForceExcitationTargetTerm () {
      super ();
      setWeight (DEFAULT_WEIGHT);
   }

   public ForceExcitationTargetTerm (TrackingController ctrl) {
      this ();
      myController = ctrl;
   }

   public ForceExcitationTargetTerm (TrackingController ctrl, double weight) {
      super ();
      setWeight (weight);
      myController = ctrl;
   }

   // ------------------------------------------------------------- controller

   /**
    * Associates this term with a {@link TrackingController}. Not strictly
    * required when the term is added as a child of the controller, but
    * setting it explicitly avoids relying on parent traversal.
    */
   public void setController (TrackingController ctrl) {
      myController = ctrl;
   }

   @Override
   public TrackingController getController () {
      if (myController != null) {
         return myController;
      }
      return super.getController ();
   }

   /**
    * Replicates {@code TrackingController.useTrapezoidalSolver()}, which is
    * protected and therefore not reachable from this package.
    */
   protected boolean usesTrapezoidal (TrackingController ctrl) {
      switch (ctrl.getUseTrapezoidalSolver ()) {
         case 0:
            return false;
         case 1:
            return true;
         default: {
            MechSystemBase mech = ctrl.getMech ();
            if (mech instanceof MechModel) {
               return ((MechModel)mech).getIntegrator () ==
                  Integrator.Trapezoidal;
            }
            return false;
         }
      }
   }

   // --------------------------------------------------------------------

   /**
    * Adds a target pinning the excitation of {@code ex} to
    * {@code targetExcitation}. The component must already be registered as an
    * exciter on the controller; the index is resolved lazily.
    *
    * @param ex excitation component (must be on the controller's exciter list)
    * @param targetExcitation desired excitation value
    * @param weight per-target QP weight
    */
   public void addExcitationTarget (
      ExcitationComponent ex, double targetExcitation, double weight) {
      if (ex == null) {
         throw new IllegalArgumentException ("exciter is null");
      }
      Target t = new Target ();
      t.comp = ex;
      t.type = TargetType.EXCITATION;
      t.targetValue = targetExcitation;
      t.weight = weight;
      myTargets.add (t);
   }

   public void addExcitationTarget (
      ExcitationComponent ex, double targetExcitation) {
      addExcitationTarget (ex, targetExcitation, 1.0);
   }

   /**
    * Adds a target pinning the <em>force</em> produced by {@code comp} to
    * {@code targetForce}. For {@link FrameExciter}, {@link PointExciter} and
    * {@link JointActuator} the conversion to an excitation target is exact.
    * For a {@link Muscle} the tension is linearised about the current
    * operating point including state coupling; see the class javadoc.
    *
    * @param comp component to track
    * @param targetForce desired force, moment, or generalized force
    * @param weight per-target QP weight
    * @throws IllegalArgumentException if {@code comp} is an unsupported type
    */
   public void addForceTarget (
      ExcitationComponent comp, double targetForce, double weight) {
      if (comp == null) {
         throw new IllegalArgumentException ("component is null");
      }
      Target t = new Target ();
      t.comp = comp;
      t.targetValue = targetForce;
      t.weight = weight;

      if (comp instanceof FrameExciter) {
         t.type = TargetType.PURE_FORCE;
         t.inverseScale = inverseOf (
            ((FrameExciter)comp).getMaxForce (), "FrameExciter maxForce");
      }
      else if (comp instanceof PointExciter) {
         t.type = TargetType.PURE_FORCE;
         t.inverseScale = inverseOf (
            ((PointExciter)comp).getMaxForce (), "PointExciter maxForce");
      }
      else if (comp instanceof JointActuator) {
         t.type = TargetType.PURE_FORCE;
         t.inverseScale = inverseOf (
            ((JointActuator)comp).getForceScale (), "JointActuator forceScale");
      }
      else if (comp instanceof Muscle) {
         t.type = TargetType.MUSCLE_FORCE;
      }
      else {
         throw new IllegalArgumentException (
            "addForceTarget: unsupported component type " +
            comp.getClass ().getSimpleName () +
            " (expected FrameExciter, PointExciter, JointActuator, or Muscle)");
      }
      myTargets.add (t);
   }

   public void addForceTarget (ExcitationComponent comp, double targetForce) {
      addForceTarget (comp, targetForce, 1.0);
   }

   private static double inverseOf (double s, String what) {
      if (s == 0) {
         throw new IllegalArgumentException (
            what + " is zero; cannot map force to excitation");
      }
      return 1.0 / s;
   }

   public void clearTargets () {
      myTargets.clear ();
   }

   public int numTargets () {
      return myTargets.size ();
   }

   public TargetType getTargetType (int i) {
      return myTargets.get (i).type;
   }

   public ExcitationComponent getTargetComponent (int i) {
      return myTargets.get (i).comp;
   }

   public double getTargetValue (int i) {
      return myTargets.get (i).targetValue;
   }
   
   public double getTargetWeight(int i) {
      return myTargets.get (i).weight;
   }

   /**
    * Finite-difference step used to estimate {@code dF/de} for muscle force
    * targets.
    */
   public double getFiniteDifferenceDelta () {
      return myFdDelta;
   }

   public void setFiniteDifferenceDelta (double delta) {
      if (delta <= 0) {
         throw new IllegalArgumentException ("delta must be positive");
      }
      myFdDelta = delta;
   }

   /** Most recently assembled H matrix. Exposed for testing. */
   public MatrixNd getH () {
      return myH;
   }

   /** Most recently assembled b vector. Exposed for testing. */
   public VectorNd getB () {
      return myB;
   }

   // -------------------------------------------------------------------

   protected int resolveIndex (ExcitationComponent ex, TrackingController c) {
      for (int i = 0; i < c.numExciters (); i++) {
         if (c.getExciter (i) == ex) {
            return i;
         }
      }
      return -1;
   }

   /**
    * Estimates {@code dF/de} for a muscle at its current length and length
    * rate, differencing about zero excitation because that is the operating
    * point the excitation response linearises about in non-incremental mode.
    */
   protected double computeDFde (Muscle m, double eRef) {
      double l = m.getLength ();
      double ldot = m.getLengthDot ();
      double F0 = m.computeF (l, ldot, eRef);
      double F1 = m.computeF (l, ldot, eRef + myFdDelta);
      return (F1 - F0) / myFdDelta;
   }

   /**
    * Warns once if a muscle force target lies below the tension the muscle
    * already develops passively. Activation is additive, so such a target can
    * never be reached and the QP will silently saturate at the lower
    * excitation bound.
    */
   protected void checkReachable (Target t, Muscle m) {
      if (t.warnedUnreachable) {
         return;
      }
      double passive = m.computeF (m.getLength (), m.getLengthDot (), 0);
      if (t.targetValue < passive) {
         t.warnedUnreachable = true;
         System.err.printf (
            "ForceExcitationTargetTerm: force target %.4f for muscle '%s' is "
            + "below its passive tension %.4f; activation is additive so this "
            + "target is unreachable and the solution will saturate.%n",
            t.targetValue, ComponentUtils.getPathName (m), passive);
      }
   }

   /**
    * Assembles the state-coupled row for one muscle force target, exactly
    * mirroring {@code ForceEffectorTerm.updateHb} for a single force
    * component, then folds in the direct activation term.
    * <p>
    * Each muscle is given its own freshly created Jacobian and block index 0.
    * This sidesteps the fact that {@code AxialSpring.addForcePosJacobian}
    * returns {@code bi++} (post-increment, so the block index never actually
    * advances), which would make several axial targets sharing one Jacobian
    * collide.
    */
   protected void buildMuscleRow (
      int row, Target t, TrackingController ctrl, double h,
      boolean trapezoidal, VectorNd u0, int numex) {

      Muscle m = (Muscle)t.comp;
      checkReachable (t, m);

      MechSystemBase mech = ctrl.getMech ();
      ForceTargetComponent fcomp = m;
      final boolean staticOnly = false; // keep velocity dependence

      SparseBlockMatrix Jf = mech.createVelocityJacobian ();

      myF0.setSize (1);
      myF0.setZero ();

      // velocity contribution, and the trapezoidal half-step position term
      fcomp.addForceVelJacobian (Jf, -1.0, 0);
      mech.reduceVelocityJacobian (Jf);
      if (trapezoidal) {
         fcomp.addForcePosJacobian (Jf, h / 2, staticOnly, 0);
         mech.reduceVelocityJacobian (Jf);
      }
      Jf.mul (myF0, myCurVel, Jf.rowSize (), myCurVel.size ());
      myF0.negate ();

      // main position term
      fcomp.addForcePosJacobian (Jf, -h, staticOnly, 0);
      mech.reduceVelocityJacobian (Jf);

      // fold in the baseline (zero-excitation) velocity response
      Jf.mulAdd (myF0, u0, Jf.rowSize (), u0.size ());

      // state-coupled columns: Hf[:,j] = Jf * Hu[:,j]
      myHfj.setSize (1);
      for (int j = 0; j < numex; j++) {
         Jf.mul (myHfj, ctrl.getHuCol (j), Jf.rowSize (), u0.size ());
         myH.set (row, j, myHfj.get (0));
      }

      // passive force error, matching core's convention
      myMinf.setSize (1);
      fcomp.getForce (myMinf, staticOnly);
      double forceError = myMinf.get (0) - t.targetValue;
      myB.set (row, forceError - myF0.get (0));

      // Direct activation term. Core's Jacobian carries the negative of the
      // excitation-induced force change, so the activation slope enters with
      // a minus sign on the muscle's own column.
      double dFde = computeDFde (m, 0.0);
      myH.add (row, t.cachedIndex, -dFde);
   }

   /**
    * Rebuilds {@link #myH} and {@link #myB} for the current step.
    */
   public void updateHb (TrackingController ctrl, double t0, double t1) {
      int numex = ctrl.numExciters ();
      int nrow = myTargets.size ();

      myH.setSize (nrow, numex);
      myB.setSize (nrow);
      myH.setZero ();
      myB.setZero ();
      if (nrow == 0 || numex == 0) {
         return;
      }

      double h = TimeBase.round (t1 - t0);
      boolean incremental = ctrl.getComputeIncrementally ();
      boolean trapezoidal = usesTrapezoidal (ctrl);

      // resolve indices once
      for (Target t : myTargets) {
         if (t.cachedIndex < 0) {
            t.cachedIndex = resolveIndex (t.comp, ctrl);
            if (t.cachedIndex < 0) {
               throw new IllegalStateException (
                  "ForceExcitationTargetTerm: component '" +
                  ComponentUtils.getPathName (t.comp) +
                  "' is not registered as an exciter on controller '" +
                  ctrl.getName () + "'");
            }
         }
      }

      // state needed by the muscle rows
      boolean haveMuscle = false;
      for (Target t : myTargets) {
         if (t.type == TargetType.MUSCLE_FORCE) {
            haveMuscle = true;
            break;
         }
      }
      VectorNd u0 = null;
      if (haveMuscle) {
         u0 = ctrl.getU0 ();
         myCurVel.setSize (u0.size ());
         ctrl.getMech ().getActiveVelState (myCurVel);
      }

      VectorNd curEx = null;
      if (incremental) {
         curEx = new VectorNd (numex);
         ctrl.getExcitations (curEx, 0);
      }

      for (int i = 0; i < nrow; i++) {
         Target t = myTargets.get (i);
         switch (t.type) {
            case EXCITATION:
            case PURE_FORCE: {
               // exact: F = e * scale, so the row is an indicator
               double eTarget = (t.type == TargetType.EXCITATION)
                  ? t.targetValue
                  : t.targetValue * t.inverseScale;
               if (incremental) {
                  eTarget -= curEx.get (t.cachedIndex);
               }
               myH.set (i, t.cachedIndex, 1.0);
               myB.set (i, eTarget);
               break;
            }
            case MUSCLE_FORCE: {
               buildMuscleRow (i, t, ctrl, h, trapezoidal, u0, numex);
               break;
            }
         }
      }

      // Normalisation. Unlike ForceEffectorTerm, whose rows are all forces in
      // the same units, this term mixes dimensionless excitation rows
      // (magnitude 1) with force rows whose magnitude is set by dF/de and can
      // easily be two orders larger. A single Frobenius norm over the whole
      // matrix would let the force rows swamp the excitation rows and make the
      // per-target weights meaningless. Normalising each row separately keeps
      // the residuals dimensionless, so a weight means the same thing whether
      // the row tracks an excitation, a force in N, or a moment in Nm.
      if (ctrl.getNormalizeCostTerms ()) {
         for (int i = 0; i < nrow; i++) {
            double rn = 0;
            for (int j = 0; j < numex; j++) {
               double v = myH.get (i, j);
               rn += v * v;
            }
            rn = Math.sqrt (rn);
            if (rn > 1e-12) {
               double s = 1.0 / rn;
               for (int j = 0; j < numex; j++) {
                  myH.set (i, j, myH.get (i, j) * s);
               }
               myB.set (i, myB.get (i) * s);
            }
         }
      }
      VectorNd weights = new VectorNd (nrow);
      for (int i = 0; i < nrow; i++) {
         weights.set (i, myTargets.get (i).weight);
      }
      myH.mulDiagonalLeft (weights);
      mulElements (myB, weights, myB);

      if (myWeight >= 0) {
         myH.scale (myWeight);
         myB.scale (myWeight);
      }
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public void getQP (MatrixNd Q, VectorNd p, double t0, double t1) {
      TrackingController ctrl = getController ();
      if (ctrl == null || myTargets.isEmpty ()) {
         return;
      }
      updateHb (ctrl, t0, t1);
      if (myH.rowSize () > 0) {
         computeAndAddQP (Q, p, myH, myB);
      }
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public int getTerm (
      MatrixNd A, VectorNd b, int rowoff, double t0, double t1) {
      TrackingController ctrl = getController ();
      if (ctrl != null && !myTargets.isEmpty ()) {
         updateHb (ctrl, t0, t1);
         A.setSubMatrix (rowoff, 0, myH);
         b.setSubVector (rowoff, myB);
         rowoff += myH.rowSize ();
      }
      return rowoff;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public int numConstraints (int qpsize) {
      return myTargets.size ();
   }
}
