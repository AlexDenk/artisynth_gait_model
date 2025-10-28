package artisynth.models.gait_model;

import java.awt.Color;

import artisynth.core.inverse.ExciterComp;
import artisynth.core.inverse.FrameExciter;
import artisynth.core.inverse.PointExciter;
import artisynth.core.inverse.TrackingController;
import artisynth.core.mechmodels.ExcitationComponent;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MultiPointMuscle;
import artisynth.core.mechmodels.Muscle;
import artisynth.models.gait_model.CoordinateActuator;

import maspack.render.RenderProps;
import maspack.util.DoubleInterval;

/**
 * The motion target controller is a subclass of {@link TrackingController} with
 * additional functionalities for the {@link CoordinateActuator}.
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
public class MotionTargetController extends TrackingController {
   private static boolean DEFAULT_CONFIG_EXCITATION_COLORING = true;
   private boolean myConfigExcitationColoring =
      DEFAULT_CONFIG_EXCITATION_COLORING;

   public MotionTargetController () {

   }

   public MotionTargetController (MechModel mech, String name) {
      super ();
      setMech (mech);
      setName (name);
      this.initComponents ();
   }

   public void initialize (double t0) {
      super.initialize (t0);
   }

   public void apply (double t0, double t1) {
      super.apply (t0, t1);
   }

   /**
    * Adds an excitation component to the set of exciters that can be used by
    * the controller.
    * 
    * @param weight
    * regularization weight to be applied to the exciter
    * @param ex
    * exciter to add
    */
   public void addExciter (double weight, ExcitationComponent ex) {
      ExciterComp ecomp = new ExciterComp (ex, weight);
      myExciters.add (ecomp);
      // keep size of myExcitations synced with number of exciters
      myExcitations.append (0);

      if (ex instanceof MultiPointMuscle) {
         MultiPointMuscle m = (MultiPointMuscle)ex;
         if (myConfigExcitationColoring && m.getExcitationColor () == null) {
            RenderProps.setLineColor (m, Color.WHITE);
            m.setExcitationColor (Color.RED);
         }
      }
      else if (ex instanceof Muscle) {
         Muscle m = (Muscle)ex;
         if (myConfigExcitationColoring && m.getExcitationColor () == null) {
            RenderProps.setLineColor (m, Color.WHITE);
            m.setExcitationColor (Color.RED);
         }
      }
      else if (ex instanceof PointExciter || ex instanceof FrameExciter) {
         ecomp.setExcitationBounds (new DoubleInterval (-1, 1));
      }
      else if (ex instanceof CoordinateActuator) {
         ecomp.setExcitationBounds (new DoubleInterval (-1, 1));
      }
   }
}