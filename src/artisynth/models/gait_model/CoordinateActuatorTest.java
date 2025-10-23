package artisynth.models.gait_model;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import artisynth.core.inverse.CoordinateActuator;
import artisynth.core.mechmodels.HingeJoint;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.probes.DataFunction;
import artisynth.core.probes.NumericMonitorProbe;
import artisynth.core.workspace.RootModel;
import maspack.interpolation.Interpolation;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.VectorNd;
import maspack.spatialmotion.SpatialInertia;
import maspack.spatialmotion.Twist;
import maspack.spatialmotion.Wrench;
import maspack.util.Clonable;

public class CoordinateActuatorTest extends RootModel {
   MechModel mech;
   RigidBody box;
   HingeJoint joint;
   CoordinateActuator exciter;

   public class BodyMovementFunction implements DataFunction, Clonable {
      RigidBody body;

      public BodyMovementFunction () {
         new BodyMovementFunction (null);
      }

      public BodyMovementFunction (RigidBody body) {

      }

      public Object clone () throws CloneNotSupportedException {
         return super.clone ();
      }

      @Override
      public void eval (VectorNd vec, double t, double trel) {
         // transform from body to world coordinates
         RigidTransform3d Xbw = box.getPose ();
         // external force (world coordinates)
         Wrench fx = box.getForce ();
         // total applied force in body coordinates
         Wrench fb = new Wrench ();
         // rotate fx to body coordinates
         fb.inverseTransform (Xbw.R, fx);

         // body velocity in body coordinates
         Twist vb = box.getVelocity ();
         // coriolis force (body coordinates)
         Wrench fc = new Wrench ();
         // spatial inertia for the body
         SpatialInertia M = box.getEffectiveInertia ();
         // compute body coriolis force
         M.bodyCoriolisForce (fc, vb);

         // spatial acceleration (body coodinates)
         Twist acc = new Twist ();
         // add coriolis force to body force
         fb.add (fc);
         // solve fb = M acc
         M.mulInverse (acc, fb);
         
         // set output variables
         vec.set (0, acc.get (5));
         vec.set (1, vb.get (5));

         // Provide feedback in the console
         System.out.println ("--------------" + t + "-------------------");
         // alpha (omega dot) according to theory:
         // M = I*alpha --> alpha = I^-1*M, with
         //
         //     |0  |   |0.167 0 0|   |alpha_x|
         // M = |0  | = |0 0.167 0| * |alpha_y|
         //     |0.1|   |0 0 0.167|   |alpha_z|
         //
         // --> alpha_x,y = 0 and
         // alpha_z = 0.1 / 0.167 = 0.6
         System.out.println ("Expected angular acceleration: 0.6 rad/s^2");
         System.out
            .println (
               "Actual angular acceleration: " + acc.get (5) + " rad/s^2");
         if (t == 1 && vb.get (5) == 0.6) {
            System.out.println ("-----------------------------------------");
            System.out
               .println (
                  "Expected angular velocity after integration with t = 1s: 0.6 rad/s");
            System.out
               .println ("Actual angular velocity: " + vb.get (5) + " rad/s");
            System.out.println("\nPassed\n");
         }
      }
   }

   public CoordinateActuatorTest () {

   }

   public CoordinateActuatorTest (String name) throws IOException {
      super (name);
   }

   public static void main (String[] args) throws IOException {

   }

   public void build (String[] args) throws IOException {
      mech = new MechModel ();
      mech.setName ("CoordinateActuatorTest");
      mech.setGravity (0, 0, 0);
      addModel (mech);
      // box
      box = RigidBody.createBox ("box", 1, 1, 1, 1);
      box.setPose (new RigidTransform3d (0, 0, 0));
      mech.addRigidBody (box);
      // joint to world
      joint = new HingeJoint (box, new RigidTransform3d (0, 0, 0));
      joint.setName ("joint");
      mech.add (joint);
      // coordinate actuator
      exciter = new CoordinateActuator ("theta_reserve", joint, "theta", 1);
      exciter.setExcitation (0.1);
      mech.add (exciter);
      // test for angular velocity after 1 s
      NumericMonitorProbe probe =
         new NumericMonitorProbe (2, getMaxStepSize ());
      probe.setModel (mech);
      probe.setName ("box movement");
      List<String> labels = new ArrayList<String> ();
      labels.add ("acceleration");
      labels.add ("velocity");
      probe.setLegendLabels (labels);
      probe.setInterpolationOrder (Interpolation.Order.Cubic);
      BodyMovementFunction f = new BodyMovementFunction (box);
      probe.setDataFunction (f);
      addOutputProbe (probe);
      // stop simulation after 1 s
      addBreakPoint (1);
   }
}