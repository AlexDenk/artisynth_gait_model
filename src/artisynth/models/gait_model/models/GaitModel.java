package artisynth.models.gait_model.models;

import java.awt.Color;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Objects;
import java.util.Set;

import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.JLabel;

import artisynth.core.femmodels.FemElement3dBase;
import artisynth.core.femmodels.FemModel.Ranging;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.gui.ControlPanel;
import artisynth.core.inverse.ConnectorForceRenderer;
import artisynth.core.inverse.DampingTerm;
import artisynth.core.inverse.ForceTarget;
import artisynth.core.inverse.InverseManager;
import artisynth.core.inverse.InverseManager.ProbeID;
import artisynth.core.inverse.L2RegularizationTerm;
import artisynth.core.inverse.MotionTargetTerm;
import artisynth.core.inverse.TargetPoint;
import artisynth.core.inverse.TrackingController;
import artisynth.core.materials.AxialMaterial;
import artisynth.core.materials.EquilibriumAxialMuscle;
import artisynth.core.materials.LinearMaterial;
import artisynth.core.mechmodels.CollisionBehaviorList;
import artisynth.core.mechmodels.CollisionManager;
import artisynth.core.mechmodels.ExcitationComponent;
import artisynth.core.mechmodels.Frame;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.JointActuator;
import artisynth.core.mechmodels.JointBase;
import artisynth.core.mechmodels.JointBasedMovingMarker;
import artisynth.core.mechmodels.JointConditionalMarker;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MechSystemSolver;
import artisynth.core.mechmodels.MechSystemSolver.Integrator;
import artisynth.core.mechmodels.MechSystemSolver.PosStabilization;
import artisynth.core.mechmodels.MeshComponentList;
import artisynth.core.mechmodels.MotionTargetComponent;
import artisynth.core.mechmodels.MultiPointMuscle;
import artisynth.core.mechmodels.MuscleComponent;
import artisynth.core.mechmodels.MuscleExciter;
import artisynth.core.mechmodels.PlanarConnector;
import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.PointList;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.mechmodels.RigidBody.InertiaMethod;
import artisynth.core.mechmodels.RigidMeshComp;
import artisynth.core.modelbase.ComponentList;
import artisynth.core.modelbase.ComponentListView;
import artisynth.core.modelbase.ComponentUtils;
import artisynth.core.modelbase.Controller;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.modelbase.MonitorBase;
import artisynth.core.modelbase.RenderableComponentList;
import artisynth.core.opensim.OpenSimParser;
import artisynth.core.opensim.components.ForceSpringBase;
import artisynth.core.opensim.customjoint.OpenSimCustomJoint;
import artisynth.core.probes.DataFunction;
import artisynth.core.probes.IKProbe;
import artisynth.core.probes.IKSolver;
import artisynth.core.probes.MarkerMotionData;
import artisynth.core.probes.NumericControlProbe;
import artisynth.core.probes.NumericDifferenceProbe;
import artisynth.core.probes.NumericInputProbe;
import artisynth.core.probes.NumericMonitorProbe;
import artisynth.core.probes.NumericOutputProbe;
import artisynth.core.probes.NumericProbeBase;
import artisynth.core.probes.PositionInputProbe;
import artisynth.core.probes.Probe;
import artisynth.core.probes.VelocityInputProbe;
import artisynth.core.renderables.ColorBar;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.RootModel;

import artisynth.models.gait_model.helper_classes.CoordinateData;
import artisynth.models.gait_model.helper_classes.CustomTRCReader;
import artisynth.models.gait_model.helper_classes.ForceData;
import artisynth.models.gait_model.helper_classes.ForceExcitationTargetTerm;
import artisynth.models.gait_model.helper_classes.MOTReader;
import artisynth.models.gait_model.helper_classes.MarkerMapping;
import artisynth.models.gait_model.helper_classes.MotionTargetController;
import artisynth.models.gait_model.helper_classes.FeasibleQPSolver;

import maspack.function.Diff1Function1x1;
import maspack.function.ScaledDiff1Function1x1;
import maspack.geometry.Face;
import maspack.geometry.Vertex3d;
import maspack.interpolation.CubicHermiteSpline1d;
import maspack.interpolation.Interpolation;
import maspack.matrix.AxisAlignedRotation;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;
import maspack.render.IsRenderable;
import maspack.render.RenderList;
import maspack.render.RenderProps;
import maspack.render.Renderer;
import maspack.render.Renderer.FaceStyle;
import maspack.render.Renderer.LineStyle;
import maspack.render.Renderer.Shading;
import maspack.render.Viewer.RotationMode;
import maspack.render.GL.GLViewer;
import maspack.spatialmotion.Wrench;
import maspack.util.Clonable;
import maspack.util.DoubleInterval;
import maspack.util.ListView;
import maspack.util.PathFinder;

/**
 * This class contains the lower-limb gait model based on the model by Tim Dorn
 * et al. The .osim file is imported and experimental boundary conditions
 * applied automatically.
 * <p>
 * To run the inverse kinematics of the model, set myUseIKSolely to true or call
 * setUseIKSolely(true) externally to this class. To remove the muscles from the
 * model, set myUseCASolely to true or call setUseCASolely(true) externally to
 * this class.
 * <p>
 * Also the model provides API to adjust components externally, e.g. via an
 * external MATLAB process. This includes setting marker and exciter weights,
 * grouping muscles and saving all probes.
 * 
 * @author Alexander Graf Copyright (c) 2026
 * <p>
 * University of Duisburg-Essen
 * <p>
 * Chair of Mechanics and Robotics
 * <p>
 * alexander.graf-lmr@uni-due.de
 */

public class GaitModel extends RootModel {
   // ----------------------------Instance Fields------------------------------
   // Current mechmodel
   MechModel myMech = new MechModel ();
   // All rigid bodies in the model
   RenderableComponentList<RigidBody> myBodies = null;
   // A list of all joints in the model
   RenderableComponentList<JointBase> myJoints;
   // All markers in the model
   RenderableComponentList<FrameMarker> myMarkers = null;
   // A list of all markers used for IK computations
   ArrayList<FrameMarker> myIKTargets = new ArrayList<FrameMarker> ();
   // All finite element meshes in the model
   List<FemModel3d> myMeshes = new ArrayList<FemModel3d> ();
   // A list of each muscle in the model
   List<MuscleComponent> myMuscles = new ArrayList<MuscleComponent> ();
   
   // Experimental and model marker names and weights
   MarkerMapping myMap;
   // Experimental marker trajectories
   MarkerMotionData myMotion;  
   // Calculated generalized joint angles
   CoordinateData myCoords;
   // Experimental force data
   ForceData myForces;
   // IK Probe + solver for marker weight overrides
   IKProbe myIKProbe;
   // Experimental marker positions to recompute IK
   NumericInputProbe myExpTargetProbe;
   // Residual computation from IK solves
   NumericDifferenceProbe myErrProbe;
   
   // Name of the current working directory
   String myName = null;
   // Scale of the model. (mm = 1000, or m = 1)
   static final double DEFAULT_SCALE = 1.0;
   double myScale = DEFAULT_SCALE;
   // Used to flag whether model runs in debug mode
   static final boolean DEFAULT_DEBUG = true;
   boolean myDebug = DEFAULT_DEBUG;
   // Used to flag whether inverse kinematics is used during forward dynamics
   static final boolean DEFAULT_FILTER_FD_WITH_IK = true;
   boolean myFilterFDWithIK = DEFAULT_FILTER_FD_WITH_IK;
   // Used to flag whether model runs entirely on IK
   static final boolean DEFAULT_USE_IK_SOLELY = false;
   boolean myUseIKSolely = DEFAULT_USE_IK_SOLELY;
   // Used to flag whether model runs only with coordinate actuators
   static final boolean DEFAULT_USE_CA_SOLELY = false;
   boolean myUseCAsSolely = DEFAULT_USE_CA_SOLELY;
   // Used to flag whether ground reaction forces are used during forward
   // dynamics
   static final boolean DEFAULT_USE_GRF = true;
   boolean myUseGRF = DEFAULT_USE_GRF;
   // Used to flag whether muscles should be grouped or not
   static final boolean DEFAULT_GROUP_MUSCLES = false;
   boolean myGroupMuscles = DEFAULT_GROUP_MUSCLES;
   // Used to flag whether excitation targets are used or not
   static final boolean DEFAULT_USE_EX_TARGET_TERM = false;
   boolean myUseExTargetTerm = DEFAULT_USE_EX_TARGET_TERM;
   
   // ----------------------------Nested classes-------------------------------
   /**
    * Subclass to provide a data function for {@link NumericControlProbe}s that
    * computes the current ground reaction moment with respect to both calcanei
    * frames. The computed wrench is applied to both frames and written to a
    * seperate output file. Current ground reaction forces are rendered with a
    * scale of 1/1000 at the current application point, i.e. both calcanei
    * frames. Current ground reaction moments are computes as:
    * <p>
    * {@code M = s x F + T} with:
    * <p>
    * <table summary="">
    * <tbody>
    *   <tr><td>s</td><td>Distance vector between force plate COP and calcanei frame</td></tr>
    *   <tr><td>F</td><td>Ground reaction force from force plate</td></tr>
    *   <tr><td>T</td><td>Ground reaction moment from force plate</td></tr>
    * </tbody>
    * </table>
    */
   public class GroundReactionMomentFunction
   implements DataFunction, Clonable, IsRenderable {
      // Frame used for moment arm calculation
      protected Frame myFrame;
      // cop identifier (left or right)
      protected String mySide;
      // PrintWriter to write computed wrenches to a file
      protected PrintWriter writer;
      // Name of the file, where computed wrenches are written to
      protected String msgName;
      // Path to the file, where computed wrenches are written to
      protected String msgPath;
      // Rendering utility variables for scaling and positioning
      protected double myScale;
      protected Vector3d copPos = new Vector3d (0, 0, 0);
      protected Vector3d grfEndPos = new Vector3d (0, 0, 0);

      public GroundReactionMomentFunction (Frame frame, String side, double s) {
         this.myFrame = frame;
         this.mySide = side;
         this.myScale = s;
         // initialize writer
         this.msgName = myName + "/Output/" + myName + "_message_file.txt";
         this.msgPath =
            ArtisynthPath
               .getSrcRelativePath (GaitModel.class, msgName).toString ();
         try {
            writer = new PrintWriter (new FileWriter (msgPath, true));
         }
         catch (IOException e) {
            e.printStackTrace ();
         }
      }

      public Object clone () throws CloneNotSupportedException {
         return super.clone ();
      }

      @Override
      public void eval (VectorNd vec, double t, double trel) {
         if (t <= 0) {
            writeHeaderToFile();
         }
         Vector3d ref = myFrame.getPosition ();
         int tframe = myForces.getFrame (t);
         // calculate moment arm from current calcn position to cop
         copPos = myForces.getData (tframe, mySide + " COP");
         Vector3d arm = new Vector3d ();
         arm.sub (copPos, ref);
         // calculate resulting moment
         Vector3d grf = new Vector3d ();
         vec.getSubVector (new int[] { 0, 1, 2 }, grf);
         Vector3d grm = new Vector3d ();
         vec.getSubVector (new int[] { 3, 4, 5 }, grm);
         Vector3d momRes = new Vector3d ();
         momRes.cross (arm, grf);
         momRes.add (grm);
         // apply wrench
         Wrench wrench =
            new Wrench (
               vec.get (0), vec.get (1), vec.get (2), momRes.x, momRes.y,
               momRes.z);
         myFrame.setExternalForce (wrench);
         // write wrench to message file
         writeDataToFile (wrench);
         // calculate endpoint of grf vector for rendering
         grfEndPos.scaledAdd (0.001 * myScale, grf, copPos);
      }

      private void writeDataToFile (Wrench wrench) {
         StringBuilder message = new StringBuilder ();
         message
            .append ("\nCOMPUTED WRENCH FOR: ")
            .append (myFrame.getName ().toUpperCase ()).append ("\n");
         message.append (wrench.toString ("%.3f")).append ("\n");
         writer.print (message);
         writer.flush ();
         message.delete (0, message.length ());
      }

      private void writeHeaderToFile () {
         StringBuilder header = new StringBuilder ();
         header
            .append (
               "%%------------------- WRENCH HISTORY FILE --------------------%%\n")
            .append (
               "%% Author: Alexander Graf, Copyright (c) 2026                 %%\n")
            .append (
               "%% (UDE) University of Duisburg-Essen                         %%\n")
            .append (
               "%% Chair of Mechanics and Robotics                            %%\n")
            .append (
               "%% alexander.graf-lmr@uni-due.de                              %%\n")
            .append (
               "%%------------------------------------------------------------%%\n");
         writer.print (header);
         writer.flush ();
      }

      @Override
      public void prerender (RenderList list) {
      }

      @Override
      public void render (Renderer renderer, int flags) {
         if (!copPos.equals (new Vector3d (0, 0, 0))
         || !grfEndPos.equals (new Vector3d (0, 0, 0))) {
            renderer.setColor (Color.GRAY.brighter ());
            renderer.drawSphere (copPos, 0.01 * myScale);
            renderer.drawArrow (copPos, grfEndPos, 0.01 * myScale, false);
         }
      }

      @Override
      public void updateBounds (Vector3d pmin, Vector3d pmax) {
      }

      @Override
      public int getRenderHints () {
         return 0;
      }
   }

   /**
    * Subclass to provide a data function for {@link NumericMonitorProbe}s that
    * computes the current generated force of any {@link ExcitationComponent}
    * with the property excitation and maxforce. Current exciter force is
    * computed as:
    * <p>
    * {@code f = a*f_max}
    */
   public class ExciterForceFunction implements DataFunction, Clonable {
      JointActuator myExciter;

      public ExciterForceFunction (ExcitationComponent e) {
         this.myExciter = (JointActuator)e;
      }

      public Object clone () throws CloneNotSupportedException {
         return super.clone ();
      }

      @Override
      public void eval (VectorNd vec, double t, double trel) {
         double ex = myExciter.getExcitation ();
         double maxForce = myExciter.getForceScale ();
         double[] force = new double[] { ex * maxForce };
         vec.set (force);
      }
   }

   /**
    * Subclass to provide a data function for {@link NumericMonitorProbe}s that
    * computes the normalized fiber length of any equilibrium exciter that has
    * the property optimalFiberLength. Normalized fiber length is computed as: 
    * <p>
    * {@code l_n = l_f/l_o}
    */
   public class NormalizedFiberLengthFunction
   implements DataFunction, Clonable {
      EquilibriumAxialMuscle mat;

      public NormalizedFiberLengthFunction (AxialMaterial mat) {
         this.mat = (EquilibriumAxialMuscle)mat;
      }

      public Object clone () throws CloneNotSupportedException {
         return super.clone ();
      }

      @Override
      public void eval (VectorNd vec, double t, double trel) {
         double lo = mat.getOptFibreLength ();
         double lm = mat.getMuscleLength ();
         double H = mat.getHeight ();
         double lf = Math.sqrt (H*H + lm*lm);
         double[] nFibLength = new double[] { lf / lo };
         vec.set (nFibLength);
      }
   }

   /**
    * Subclass to provide smoothing to {@link NumericProbeBase} objects, after
    * all data has been collected. Smoothing is applied via a moving average
    * with a window size of:
    * <p>
    * {@code wSize = 0.1 / t_max}
    * <p>
    * with t_max being the maximum step size of the current model.
    */
   public class SmoothingMonitor extends MonitorBase {
      Map<NumericProbeBase, Double> probeMap =
         new HashMap<NumericProbeBase, Double> ();
      int winSize;

      public SmoothingMonitor (ArrayList<NumericProbeBase> probes) {
         for(NumericProbeBase probe : probes) {
            Double tFinal = probe.getStopTime ();
            probeMap.put (probe, tFinal);
         }
      }

      public SmoothingMonitor () {
         
      }

      public void add (NumericProbeBase probe) {
         Double tFinal = probe.getStopTime ();
         this.probeMap.put (probe, tFinal);
      }

      public ArrayList<NumericProbeBase> getAll () {
         ArrayList<NumericProbeBase> probes =
            new ArrayList<NumericProbeBase> ();
         probeMap.forEach ( (key, value) -> {
            probes.add (key);
         });
         return probes;
      }

      @Override
      public void initialize (double t0) {
         super.initialize (t0);
         winSize = (int)(0.1 / getMaxStepSize ());
      }

      @Override
      public void apply (double t0, double t1) {
         // Get all probes, that end on t1
         Set<NumericProbeBase> keys = new HashSet<NumericProbeBase> ();
         for (Entry<NumericProbeBase, Double> entry : probeMap.entrySet ()) {
            if (Objects.equals (t1, entry.getValue ())) {
               keys.add (entry.getKey ());
            }
         }
         // Smooth all corresponding probes
         keys.forEach (key -> {
            NumericProbeBase probe = key;
            probe.smoothWithMovingAverage (winSize);
         });
      }
   }

   // -----------------------------Constructors--------------------------------
   public GaitModel () {
      
   }

   // --------------------------Static Methods---------------------------------
   public static void main (String[] args) throws IOException {
   }

   // -------------------------Instance Methods--------------------------------
   @Override
   public void build (String[] args) throws IOException {
      super.build (args);
      addMechModel (args);
      setSimulationProperties ();
      initializeOsim (myName, myScale);
      CollisionManager collMan = myMech.getCollisionManager ();
      setContactProps (collMan, myName);
      initializeIOProbes (myName, myScale);
      setRenderProps (collMan, myScale);
      writeInputToFile (myName);
   }
   
   public boolean getDebug () {
      return myDebug;
   }

   public void setDebug (boolean debug) {
      myDebug = debug;
   }
   
   /**
    * 
    * @return The names associated with each exciter of the {@link TrackingController}.
    */
   public String[] getExciterNames () {
      if (getControllers ().get ("Motion controller") == null) {
         System.out
            .println (
               "GaitModel: got no exciter names (controller not found).");
         return null;
      }
      MotionTargetController controller =
         (MotionTargetController)getControllers ().get ("Motion controller");
      String[] names = new String[controller.getExciters ().size ()];
      for (int i = 0; i < controller.getExciters ().size (); i++) {
         names[i] = controller.getExciter (i).getName ();
      }
      return names;
   }
   
   /**
    * 
    * @return the weights associated with each exciter of the {@link TrackingController}.
    */
   public double[] getExciterWeights () {
      if (getControllers ().get ("Motion controller") == null) {
         System.out
         .println (
            "GaitModel: got no exciter weights (controller not found).");
         return null;
      }
      MotionTargetController controller =
         (MotionTargetController)getControllers ().get ("Motion controller");
      double[] weights = new double[controller.getExciters ().size ()];
      int k = 0;
      for (ExcitationComponent ex : controller.getExciters ()) {
         weights[k] = controller.getExcitationWeight (ex);
         k++;
      }
      return weights;
   }
   
   /**
    * Sets and overrides the exciter weights used by the
    * {@link TrackingController} for each valid pair of exciter name and weight.
    * 
    * @param names
    * of the exciters to be overriden.
    * @param weights
    * new weight corresponding to each exciter.
    */
   public void setExciterWeights (String[] names, double[] weights) {
      if (names.length != weights.length) {
         System.out
            .println (
               "GaitModel: exciter weights not set (dimensions mismatch).");
         return;
      }
      if (getControllers ().get ("Motion controller") == null) {
         System.out
            .println (
               "GaitModel: exciter weights not set (controller not found).");
         return;
      }
      MotionTargetController controller =
         (MotionTargetController)getControllers ().get ("Motion controller");
      for (int i = 0; i < weights.length; i++) {
         for (ExcitationComponent ex : controller.getExciters ()) {
            if (ex.getName ().equals (names[i])) {
               controller.setExcitationWeight (ex, weights[i]);
            }
         }
      }
   }

   public boolean getFilterFDWithIK () {
      return myFilterFDWithIK;
   }

   public void setFilterFDWithIK (boolean filterWithIK) {
      myFilterFDWithIK = filterWithIK;
   }
   
   public boolean getGroupMuscles() {
      return myGroupMuscles;
   }
   
   public void setGroupMuscles(boolean groupMuscles) {
      myGroupMuscles = groupMuscles;
   }

   /**
    * Sets and overrides the motion target weights used by the
    * {@link TrackingController} and {@link IKSolver}. Recomputes IK positions
    * and associated tracking error against experimental marker trajectories.
    * 
    * @param names
    * @param weights
    */
   public void setMarkerWeights (String[] names, double[] weights) {
      // Override each motion target weight in the TrackingController
      MotionTargetController controller =
         (MotionTargetController)getControllers ().get ("Motion controller");
      ArrayList<MotionTargetComponent> targets = controller.getMotionTargets ();
      VectorNd w = controller.getMotionTargetWeights ();
      for (int k = 0; k < myIKTargets.size (); k++) {
         String mkrName = myIKTargets.get (k).getName ();
         for (int i = 0; i < weights.length; i++) {
            if (mkrName.equals (names[i])) {
               controller.setMotionTargetWeight (targets.get (k), weights[i]);
               w.set (k, weights[i]);
            }
         }
      }
      // Recompute IK positions for new achievable targets
      IKSolver solver = new IKSolver (myMech, myIKTargets, w);
      solver.setMaxIterations (100);
      PositionInputProbe probe =
         solver
            .createMarkerPositionProbe (
               "IK target positions", myExpTargetProbe, true, -1);
      solver.solve (myExpTargetProbe.getNumericList ().getFirst ().v);
      // Recompute tracking residuals
      myErrProbe.setProbes (myExpTargetProbe, probe);
      myErrProbe.updateData (getMaxStepSize());
      // Override IK marker weights used during replay
      myIKProbe.getSolver ().setMarkerWeights (w);
      myIKProbe.setData (probe, true);
   }

   public boolean getUseCASolely () {
      return myUseCAsSolely;
   }

   public void setUseCASolely (boolean useCA) throws IOException {
      myUseCAsSolely = useCA;
      if (useCA == true)
         setupCoordinateActuatorRun ();
   }

   public boolean getUseExTargetTerm () {
      return myUseExTargetTerm;
   }

   public void setUseExTargetTerm (boolean useTargetTerm) {
      myUseExTargetTerm = useTargetTerm;
   }

   public boolean getUseGRF () {
      return myUseGRF;
   }

   public void setUseGRF (boolean useGRF) {
      myUseGRF = useGRF;
   }

   public boolean getUseIKSolely () {
      return myUseIKSolely;
   }

   public void setUseIKSolely (boolean useIKSolely) throws IOException {
      myUseIKSolely = useIKSolely;
      if (useIKSolely == true)
         setupInverseKinematicsRun ();
   }

   public double getScale () {
      return myScale;
   }

   public void setScale (double scale) {
      myScale = scale;
      myMech.scaleDistance (scale);
      myMech.scaleMass (scale);
   }

   @Override
   public void prerender (RenderList list) {
      super.prerender (list);
      // Synchronize color bar/values in case they are changed. Do this *after*
      // super.prerender(), in case values are changed there.
      for (int i = 0; i < myMeshes.size (); i++) {
         ColorBar cbar =
            (ColorBar)(renderables ()
               .get (myMeshes.get (i).getName () + "_colorbar"));
         cbar.setColorMap (myMeshes.get (i).getColorMap ());
         DoubleInterval range = myMeshes.get (i).getStressPlotRange ();
         cbar.updateLabels (range.getLowerBound (), range.getUpperBound ());
      }
   }
   
   /**
    * Saves the tracking error output probe from IK computations
    */
   public void saveIKPositionError () {
      RenderableComponentList<Probe> probes = getOutputProbes ();
      probes.forEach (p -> {
         if (p.getName ().equals ("IK position error"))
            try {
               p.save ();
            }
            catch (IOException e) {
               e.printStackTrace ();
            }
      });
   }
   
   /**
    * Saves all output probes of the current active model to their designated
    * files.
    */
   public void saveAllOutputProbes () {
      RenderableComponentList<Probe> probes = getOutputProbes ();
      probes.forEach (probe -> {
         try {
            probe.save ();
         }
         catch (IOException e) {
            e.printStackTrace ();
         }
      });
   }

   
   
   /**
    * Sets the rendering properties of every body connector within the root
    * model.
    * 
    * @param scale
    * Unit scaling factor for the current model (m = 1, mm = 1000)
    */
   public void setConnectorRenderProps (int scale) {
      if (scale < 0)
         throw new IllegalArgumentException ("Scale factor must be > 0");
      List<PlanarConnector> con = new ArrayList<PlanarConnector> ();
      myMech.bodyConnectors ().forEach (c -> {
         con.add ((PlanarConnector)c);
      });
      List<ConnectorForceRenderer> rend =
         new ArrayList<ConnectorForceRenderer> ();
      RenderProps props = new RenderProps ();
      props.setLineStyle (LineStyle.CYLINDER);
      props.setLineRadius (25 * scale / 1000);
      props.setLineColor (Color.GREEN);
      con.forEach (c -> {
         RenderProps.setSphericalPoints (c, 0.01 * scale, Color.CYAN);
         RenderProps.setFaceStyle (c, FaceStyle.FRONT);
         rend.add (new ConnectorForceRenderer (c));
         int end = rend.size ();
         rend.get (end - 1).setRenderProps (props);
         rend.get (end - 1).setArrowSize (0.001 * scale);
         addMonitor (rend.get (end - 1));
      });
   }

   /**
    * Sets the rendering properties of every collision response within the root
    * model.
    * 
    * @param coll
    * {@link CollisionManager}
    * @param scale
    * Unit scaling factor for the current model (m = 1, mm = 1000)
    */
   public void setContactRenderProps (CollisionManager coll, double scale) {
      if (coll == null)
         return;
      if (scale < 0)
         throw new IllegalArgumentException ("Scale factor must be > 0");
      // Define body - body contact render props
      coll.setDrawIntersectionPoints (true);
      coll.setDrawContactForces (true);
      coll.setDrawFrictionForces (true);
      coll.setContactForceLenScale (scale / 1000);
      RenderProps.setVisible (coll, true);
      RenderProps.setSolidArrowLines (coll, 0.01 * scale, Color.BLUE);
      RenderProps.setSphericalPoints (coll, 0.01 * scale, Color.CYAN);
   }

   /**
    * Sets the rendering properties of every model marker within the root model.
    * 
    * @param markers
    * 
    */
   public void setMarkerRendering (
      RenderableComponentList<FrameMarker> markers, double scale) {
      if (markers == null)
         return;
      markers.forEach (m -> {
         RenderProps.setPointColor (m, Color.PINK);
      });
      // Access source and target points of the motion target controller
      ComponentListView<Controller> controllers = getControllers ();
      if (controllers.size () == 0)
         return;
      MotionTargetController controller =
         (MotionTargetController)controllers.get ("Motion controller");
      controller.getTargetPoints ().forEach (c -> {
         RenderProps.setPointColor (c, Color.WHITE);
         RenderProps.setPointRadius (c, 0.01 * scale);
      });
   }

   /**
    * Sets the rendering properties of every mesh within the root model.
    * 
    * @param meshes
    * 
    * @param scale
    * Unit scaling factor for the current model (m = 1, mm = 1000)
    */
   public void setMeshRendering (List<FemModel3d> meshes, double scale) {
      if (scale < 0)
         throw new IllegalArgumentException ("Scale factor must be > 0");
      meshes.forEach (mesh -> {
         RenderProps.setLineStyle (mesh, LineStyle.LINE);
         RenderProps.setLineWidth (mesh, 1);
         RenderProps.setLineColor (mesh, Color.BLACK);
      });
   }

   /**
    * Sets the rendering properties of every muscle within the root model.
    * 
    * @param muscles
    * 
    * @param scale
    * Unit scaling factor for the current model (m = 1, mm = 1000)
    */
   public void setMuscleRenderProps (
      List<MuscleComponent> muscles, double scale) {
      if (muscles == null)
         return;
      if (scale < 0)
         throw new IllegalArgumentException ("Scale factor must be > 0");
      muscles.forEach (msc -> {
         RenderProps.setLineColor (msc, Color.RED.darker ());
         RenderProps.setShading (msc, Shading.SMOOTH);
         RenderProps.setLineRadius (msc, 0.005 * scale);
         RenderProps
            .setSpindleLines (myMech, 0.02 * scale, Color.RED.darker ());
         msc.setExcitationColor (Color.GREEN);
      });
      myMech.setMaxColoredExcitation (1.0);
   }

   /**
    * Sets the render properties of the current model regarding bodies, muscles,
    * meshes, contact and viewer props.
    * 
    * @param coll
    * {@link CollisionManager}
    * @param scale
    * Unit scaling factor for the current model (m = 1, mm = 1000)
    */
   public void setRenderProps (CollisionManager coll, double scale) {
      if (!(coll instanceof CollisionManager))
         throw new IllegalArgumentException (
            "Must use an object of class CollisionManager");
      if (scale < 0)
         throw new IllegalArgumentException ("Scale factor must be > 0");
      setContactRenderProps (coll, scale);
      setRigidBodyRenderProps (myBodies, scale);
      setMuscleRenderProps (myMuscles, scale);
      setMarkerRendering (myMarkers, scale);
      setMeshRendering (myMeshes, scale);
      setSurfaceRenderProps ();
      setViewerProps ();
   }

   public void setRigidBodyRenderProps (
      RenderableComponentList<RigidBody> bodies, double scale) {
      if (bodies == null)
         return;
      RenderProps.setShading (bodies, Shading.SMOOTH);
      RigidBody ground = (RigidBody)myMech.get ("ground");
      ground.setAxisLength (scale);
   }

   /**
    * Sets the render properties of the colour bars within the root model.
    */
   public void setSurfaceRenderProps () {
      ColorBar[] cbar = new ColorBar[myMeshes.size ()];
      for (int i = 0; i < myMeshes.size (); i++) {
         myMeshes.get (i).setSurfaceRendering (SurfaceRender.Stress);
         myMeshes.get (i).setStressPlotRanging (Ranging.Auto);
         cbar[i] = new ColorBar ();
         cbar[i].setName (myMeshes.get (i).getName () + "_colorbar");
         cbar[i].setNumberFormat ("%.2f");
         cbar[i].populateLabels (0.0, 0.1, 10);
         cbar[i].setLocation (-100, 0.1 * i, 20, 0.8);
         addRenderable (cbar[i]);
      }
   }
  
   /**
    * Sets the viewer properties of the current viewer.
    */
   public void setViewerProps () {
      GLViewer viewer = getMainViewer ();
      if (viewer != null) {
         viewer.setOrthographicView (true);
         viewer.setRotationMode (RotationMode.CONTINUOUS);
      }
      setDefaultViewOrientation (AxisAlignedRotation.X_Y);
      mergeAllControlPanels (true);
   }

   /**
    * Writes all model parameters to a {@code myName_input.txt} in the Output
    * directory under the current working directory.
    * 
    * @param myName
    * Name specifier of the current working directory
    * @throws IOException
    * if there's an issue writing to the file
    */
   public void writeInputToFile (String myName) throws IOException {
      if (!(myName instanceof String))
         throw new IllegalArgumentException ("Input must be of type String");
      String inputName = myName + "/Input/" + myName + "_input_file.txt";
      String inputPath =
         ArtisynthPath.getSrcRelativePath (this, inputName).toString ();
      try (PrintWriter writer =
         new PrintWriter (new FileWriter (inputPath, false))) {
         // Append header
         StringBuilder output = new StringBuilder ();
         output
            .append (
               "%%------------------------ INPUT FILE ------------------------%%\n")
            .append (
               "%% Author: Alexander Graf, Copyright (c) 2025                 %%\n")
            .append (
               "%% (UDE) University of Duisburg-Essen                         %%\n")
            .append (
               "%% Chair of Mechanics and Robotics                            %%\n")
            .append (
               "%% alexander.graf-lmr@uni-due.de                              %%\n")
            .append (
               "%%------------------------------------------------------------%%\n");
         MechSystemSolver solver = myMech.getSolver ();
         writeSolverInfo (output, solver);
         writePhysicsInfo (output, myMech);
         String modelPath = myName + "_scaled.osim";
         writeModelInfo (output, modelPath, myBodies);
         writeJointInfo (output, myJoints);
         writeMuscleInfo (output, myMuscles);
         writeFEMInfo (output, myMeshes);
         MotionTargetController controller =
            (MotionTargetController)getControllers ().get ("Motion controller");
         if (controller != null)
            writeProbesInfo (
               output, controller, myMotion, myMap, myForces, myMarkers);
         CollisionManager coll = myMech.getCollisionManager ();
         CollisionBehaviorList behav = coll.behaviors ();
         if (coll != null)
            writeContactInfo (output, coll, behav);
         writer.print (output.toString ());
      }
      catch (IOException ex) {
         System.err.println (ex.getMessage ());
      }
   }

   // --------------------------Private Instance Methods------------------------
   /**
    * Adds output probes and panel widgets for the excitations of all exciters
    * of the MotionTargetController and also max force/moment if the model is
    * driven by frame exciters or coordinate actuators.
    * 
    * @param controller
    * {@link MotionTargetController}
    * @param monitor
    * @param start
    * start time
    * @param stop
    * stop time
    * @param step
    * step size
    */
   private void addActuatorOutputProbes (
      MotionTargetController controller, SmoothingMonitor monitor, double start,
      double stop, double step) {
      ControlPanel musclePanel = new ControlPanel ("Muscle Properties");
      controller.getExciters ().forEach (e -> {
         if (e instanceof JointActuator) {
            if (myDebug) {
               createExciterOutputProbe (e, monitor, start, stop, step);
               musclePanel
                  .addWidget (e.getName () + " excitation", e, "excitation");
               musclePanel
                  .addWidget (e.getName () + " maxForce", e, "forceScale");
            }
         }
         else if (e instanceof MuscleExciter) {
            MuscleExciter ex = (MuscleExciter)e;
            for (int i = 0; i < ex.numTargets (); i++) {
               ExcitationComponent target = ex.getTarget (i);
               if (target instanceof MuscleComponent) {
                  createProbeAndPanel (
                     target, null, "forceNorm", monitor, start, stop, step);
                  musclePanel
                     .addWidget (
                        target.getName () + " excitation", target,
                        "excitation");
                  if (myDebug)
                     createMusFbrLgthOutputProbe (
                        target, monitor, start, stop, step);
               }
            }
         }
         else {
            createProbeAndPanel (
               e, null, "forceNorm", monitor, start, stop, step);
            musclePanel
               .addWidget (e.getName () + " excitation", e, "excitation");
            if (myDebug)
               createMusFbrLgthOutputProbe (e, monitor, start, stop, step);
         }
      });
      addControlPanel (musclePanel);
   }

   /**
    * Generates a {@link NumericOutputProbe} for left and right calcaneus that
    * tracks the generated external force by
    * {@link GroundReactionMomentFunction}.
    * 
    * @param start
    * start time
    * @param stop
    * stop time
    * @param step
    * step size
    */
   private void addBodyForceOutputProbes (
      double start, double stop, double step) {
      RigidBody calcnL = myBodies.get ("calcn_l");
      createProbeAndPanel (
         calcnL, null, "externalForce", null, start, stop, step);
      RigidBody calcnR = myBodies.get ("calcn_r");
      createProbeAndPanel (
         calcnR, null, "externalForce", null, start, stop, step);
   }

   /**
    * Defines a tracking controller that calculates muscle activations based on
    * trajectories and sets its controller properties.
    * 
    * @return generated tracking controller object
    */
   private MotionTargetController addControllerAndProps () {
      MotionTargetController controller =
         new MotionTargetController (myMech, "Motion controller");
      controller.setUseKKTFactorization (false);
      controller.setComputeIncrementally (true);
      controller.setRefactorForIncremental (false);
      if (myUseCAsSolely) {
         controller.setMotionTargetAsConstraint (false);
         controller.setL2Regularization ();
      }
      else {
         controller.setMotionTargetAsConstraint (true);
         controller.setExcitationDamping ();
         controller.setL2Regularization (15);
      }
      // Adjust MotionTargetTerm properties
      MotionTargetTerm motionTerm = controller.getMotionTargetTerm ();
      motionTerm.setUsePDControl (true);
      if (myUseCAsSolely) {
         motionTerm.setChaseTime (0.001);
      }
      else {
         motionTerm.setChaseTime (0.01);
      }
      motionTerm.setKd (1e-1 / getMaxStepSize ());
      motionTerm.setKp (1 / (getMaxStepSize () * motionTerm.getChaseTime ()));
      controller.createPanel (this);
      addController (controller);
      return controller;
   }

   /**
    * Adds all model muscles and three frame exciters each for pelvis and both
    * calcanei to the controller.
    * 
    * @param controller
    * {@link MotionTargetController}
    */
   private void addExcitersToController (MotionTargetController controller) {
      // Muscles
      if (myGroupMuscles) {
         addGroupedExciters (controller);
      }
      else {
         controller.addExciters (myMuscles);
      }
      // Coordinate actuators
      HashSet<String> jointsWithSmallExcs = new HashSet<String> ();
      jointsWithSmallExcs.add ("acromial_r");
      jointsWithSmallExcs.add ("acromial_l");
      jointsWithSmallExcs.add ("elbow_r");
      jointsWithSmallExcs.add ("elbow_l");
      jointsWithSmallExcs.add ("radioulnar_r");
      jointsWithSmallExcs.add ("radioulnar_l");
      jointsWithSmallExcs.add ("radius_hand_r");
      jointsWithSmallExcs.add ("radius_hand_l");
      jointsWithSmallExcs.add ("r_scapulothoracic");
      jointsWithSmallExcs.add ("l_scapulothoracic");
      jointsWithSmallExcs.add ("r_sternoclavicular");
      jointsWithSmallExcs.add ("l_sternoclavicular");
      ArrayList<JointActuator> smallExs = new ArrayList<> ();
      ArrayList<JointActuator> reserveExs = new ArrayList<> ();
      myJoints.forEach (joint -> {
         if (jointsWithSmallExcs.contains (joint.getName ()))
            smallExs
               .addAll (JointActuator.createActuators (myMech, joint, 100));
         else
            reserveExs
               .addAll (JointActuator.createActuators (myMech, joint, 1000));
      });
      for (JointActuator ex : smallExs) {
         String name = ex.getCoordinateHandle ().getName () + "_reserve";
         ex.setName (name);
         controller.addExciter (1, ex);
         controller.setExcitationBounds (ex, -1, 1);
      }
      for (JointActuator ex : reserveExs) {
         double weight;
         if (myUseCAsSolely) {
            weight = 1;
         }
         else {
            weight = 1000;
         }
         String name = ex.getCoordinateHandle ().getName () + "_reserve";
         ex.setName (name);
         controller.addExciter (weight, ex);
         controller.setExcitationBounds (ex, -1, 1);
         if (myUseExTargetTerm) {
            if (name.contains ("hip") || name.contains ("knee")
            || name.contains ("ankle")) {
               controller.addExcitationTarget (ex, 0, 0.1);
            }
         }
      }
   }

   private void addGroupedExciters (MotionTargetController controller) {
      ArrayList<MuscleComponent> ankleDFLeft =
         getMuscles (
            new String[] { "tib_ant_l", "ext_dig_l", "ext_hal_l",
                           "per_tert_l" });
      addExciterGroupToController (controller, ankleDFLeft, "ankle_df_l");
      ArrayList<MuscleComponent> ankleDFRight =
         getMuscles (
            new String[] { "tib_ant_r", "ext_dig_r", "ext_hal_r",
                           "per_tert_r" });
      addExciterGroupToController (controller, ankleDFRight, "ankle_df_r");

      ArrayList<MuscleComponent> anklePFLeft =
         getMuscles (
            new String[] { "soleus_l", "med_gas_l", "lat_gas_l", "tib_post_l",
                           "flex_dig_l", "flex_hal_l", "per_brev_l",
                           "per_long_l" });
      addExciterGroupToController (controller, anklePFLeft, "ankle_pf_l");
      ArrayList<MuscleComponent> anklePFRight =
         getMuscles (
            new String[] { "soleus_r", "med_gas_r", "lat_gas_r", "tib_post_r",
                           "flex_dig_r", "flex_hal_r", "per_brev_r",
                           "per_long_r" });
      addExciterGroupToController (controller, anklePFRight, "ankle_pf_r");

      ArrayList<MuscleComponent> kneeExtLeft =
         getMuscles (
            new String[] { "rect_fem_l", "vas_int_l", "vas_lat_l",
                           "vas_med_l" });
      addExciterGroupToController (controller, kneeExtLeft, "knee_ext_l");
      ArrayList<MuscleComponent> kneeExtRight =
         getMuscles (
            new String[] { "rect_fem_r", "vas_int_r", "vas_lat_r",
                           "vas_med_r" });
      addExciterGroupToController (controller, kneeExtRight, "knee_ext_r");

      ArrayList<MuscleComponent> swingInitLeft =
         getMuscles (new String[] { "psoas_l", "iliacus_l" });
      addExciterGroupToController (controller, swingInitLeft, "swing_init_l");
      ArrayList<MuscleComponent> swingInitRight =
         getMuscles (new String[] { "psoas_r", "iliacus_r" });
      addExciterGroupToController (controller, swingInitRight, "swing_init_r");

      ArrayList<MuscleComponent> glutMinLeft =
         getMuscles (
            new String[] { "glut_min1_l", "glut_min2_l", "glut_min3_l" });
      addExciterGroupToController (controller, glutMinLeft, "glut_min_l");
      ArrayList<MuscleComponent> glutMinRight =
         getMuscles (
            new String[] { "glut_min1_r", "glut_min2_r", "glut_min3_r" });
      addExciterGroupToController (controller, glutMinRight, "glut_min_r");

      ArrayList<MuscleComponent> glutMedLeft =
         getMuscles (
            new String[] { "glut_med1_l", "glut_med2_l", "glut_med3_l" });
      addExciterGroupToController (controller, glutMedLeft, "glut_med_l");
      ArrayList<MuscleComponent> glutMedRight =
         getMuscles (
            new String[] { "glut_med1_r", "glut_med2_r", "glut_med3_r" });
      addExciterGroupToController (controller, glutMedRight, "glut_med_r");

      ArrayList<MuscleComponent> glutMaxLeft =
         getMuscles (
            new String[] { "glut_max1_l", "glut_max2_l", "glut_max3_l" });
      addExciterGroupToController (controller, glutMaxLeft, "glut_max_l");
      ArrayList<MuscleComponent> glutMaxRight =
         getMuscles (
            new String[] { "glut_max1_r", "glut_max2_r", "glut_max3_r" });
      addExciterGroupToController (controller, glutMaxRight, "glut_max_r");

      ArrayList<MuscleComponent> addMagLeft =
         getMuscles (new String[] { "add_mag1_l", "add_mag2_l" });
      addExciterGroupToController (controller, addMagLeft, "add_mag_l");
      ArrayList<MuscleComponent> addMagRight =
         getMuscles (new String[] { "add_mag1_r", "add_mag2_r" });
      addExciterGroupToController (controller, addMagRight, "add_mag_r");
      controller
         .addExciters (
            getMuscles (
               new String[] { "semimem_r", "semimem_l", "semiten_r",
                              "semiten_l", "bifemlh_r", "bifemlh_l",
                              "bifemsh_r", "bifemsh_l", "add_brev_r",
                              "add_brev_l", "add_long_r", "add_long_l",
                              "add_mag3_r", "add_mag3_l", "pect_r", "pect_l",
                              "gem_r", "gem_l", "peri_r", "peri_l",
                              "quad_fem_r", "quad_fem_l", "grac_r", "grac_l",
                              "sar_r", "sar_l", "tfl_r", "tfl_l", "ercspn_r",
                              "ercspn_l", "extobl_r", "extobl_l", "intobl_r",
                              "intobl_l" }));
   }

   /**
    * Groups the provided {@link MuscleComponent} objects into a
    * {@link MuscleExciter} object with a default weight of 1 and adds the
    * muscle exciter to the {@link TrackingController}.
    * 
    * @param controller
    * @param muscles
    * @param groupName
    */
   private void addExciterGroupToController (
      MotionTargetController controller, ArrayList<MuscleComponent> muscles,
      String groupName) {
      MuscleExciter groupedMuscles = new MuscleExciter ();
      groupedMuscles.setName (groupName);
      for (MuscleComponent m : muscles) {
         groupedMuscles.addTarget (m);
      }
      myMech.addMuscleExciter (groupedMuscles);
      controller.addExciter (groupedMuscles);
   }

   /**
    * Adds the data stored in {@code myForces} as individual
    * {@link NumericInputProbe} if available.
    * 
    * @param forces
    * {@link ForceData}
    * @param start
    * start time
    * @param stop
    * stop time
    * @param scale
    * Unit scaling factor (1 = m, 1000 = mm)
    */
   private void addForceInputProbes (
      ForceData forces, double start, double stop, double scale) {
      if (forces == null)
         return;
      if (myUseGRF == false)
         return;
      RigidBody calcnR = myBodies.get ("calcn_r");
      createForceInputProbe (forces, calcnR, "Right", start, stop, scale);
      RigidBody calcnL = myBodies.get ("calcn_l");
      createForceInputProbe (forces, calcnL, "Left", start, stop, scale);
   }

   /**
    * Adds output probes and panel widgets for joint angles.
    * 
    * @param start
    * start time
    * @param monitor
    * @param stop
    * stop time
    * @param step
    * step size
    */
   private void addJointAngleOutputProbes (
      double start, SmoothingMonitor monitor, double stop, double step) {
      ControlPanel jointPanel = new ControlPanel ("Joint Coordinates");
      myJoints.forEach (jt -> {
         for (int i = 0; i < jt.numCoordinates (); i++) {
            createProbeAndPanel (
               jt, jointPanel, jt.getCoordinateName (i), monitor, start, stop,
               step);
            jointPanel
               .addWidget (
                  jt.getCoordinateName (i) + " locked", jt,
                  jt.getCoordinateName (i) + "_locked");
         }
      });
      addControlPanel (jointPanel);
   }

   /**
    * Adds and ets the mech model name either from the first argument in
    * {@code args} or file dialog if {@code args} is empty.
    * 
    * @param args
    */
   private void addMechModel (String[] args) {
      if (args.length == 0) {
         JFileChooser fc = new JFileChooser ();
         myName = getNameFromFileDiaglog (fc);
      }
      else
         myName = args[0];
      myMech.setName (myName);
      addModel (myMech);
      System.out
         .println (
            "---------------------------------------------------------------");
      System.out.println ("GaitModel: loaded " + myName);
   }

   /**
    * Creates the default {@link NumericOutputProbe} of the
    * {@link MotionTargetController} and fills a control panel with all
    * corresponding properties if specified.
    * 
    * @param motion
    * {@link MarkerMotionData}
    * @param start
    * start time
    * @param stop
    * stop time
    */
   private void addNumOutputProbesAndPanel (
      MarkerMotionData motion, MotionTargetController controller, double start,
      double stop) {
      if (motion == null)
         return;
      double step = getMaxStepSize ();
      SmoothingMonitor monitor = new SmoothingMonitor ();
      monitor.setName ("Smoothing monitor");
      if (controller != null) {
         setDefaultOutputProbes (controller, start, stop);
         if (controller.isActive () == true) {
            addActuatorOutputProbes (controller, monitor, start, stop, step);
         }
      }
      addJointAngleOutputProbes (start, null, stop, step);
      if (myDebug)
         addBodyForceOutputProbes (start, stop, step);
      addMonitor (monitor);
   }

   /**
    * Defines the point targets of the {@link MotionTargetTerm} of the
    * {@link MotionTargetController} and creates a {@link NumericInputProbe} for
    * each target.
    * 
    * @param controller
    * MotionTargetController
    * @param map
    * links the model marker names to the experimental marker names and their
    * weights
    * @param motion
    * experimental marker motion data
    * @param start
    * start time
    * @param stop
    * stop time
    * @throws IOException
    * if one or multiple files don't exist
    * @throws CloneNotSupportedException 
    */
   private void addPointTargetsAndProbes (
      MotionTargetController controller, MarkerMapping map,
      MarkerMotionData motion, double start, double stop)
      throws IOException {
      if (motion == null || map == null)
         return;
      // Add markers to be targets and set their positions
      VectorNd weights = new VectorNd ();
      for (int i = 0; i < myMarkers.size (); i++) {
         FrameMarker mkr = myMarkers.get (i);
         String markerName = mkr.getName ();
         String expName = map.getExpLabelFromModel (markerName);
         if (expName != null) {
            Double weight = map.getMarkerWeight (markerName);
            weights.append (weight);
            myIKTargets.add (mkr);
            TargetPoint target = controller.addPointTarget (mkr, weight);
            // Adjust initial position of each target
            Point3d position = (Point3d)motion.getMarkerPosition (0, expName);
            target.setPosition (position);
         }
      }
      // Add the default MarkerMotionData prior to IK solving
      myExpTargetProbe =
         collectMarkerMotionData (
            controller, myIKTargets, motion, map, start, stop);
      // clone experimental targets to save them from being overriden
      NumericInputProbe targetProbe = null;
      try {
         targetProbe = (NumericInputProbe)myExpTargetProbe.clone ();
         targetProbe.setName (myExpTargetProbe.getName ());
      }
      catch (CloneNotSupportedException e) {
         e.printStackTrace ();
      }
      addInputProbe (targetProbe);
      if (myFilterFDWithIK) {
         // Generate achievable target positions by IK
         PositionInputProbe posProbe =
            createIKPositions (myIKTargets, weights, targetProbe);
         // Generate an OutputProbe for the tracking error of IK *before*
         // targetProbe gets overwritten
         myErrProbe = createIKPositionError (targetProbe, posProbe);
         addOutputProbe (myErrProbe);
         // Overwrite default marker positions with achievable ones
         targetProbe.setData (posProbe, true);
         // Generate VelocityInputProbes from PositionInputProbes
         VelocityInputProbe velProbe = createVelProbe (posProbe);
         addInputProbe (velProbe);
      }
   }

   /**
    * Effectively stores the original marker positions as specified in
    * {@code motion} to hand them to an IK solver later.
    * 
    * @param controller
    * MotionTargetController
    * @param targets
    * Collection of PointTargets
    * @param motion
    * MarkerMotionData
    * @param map
    * MarkerMapping
    * @param stop 
    * @param start 
    * @return
    */
   private NumericInputProbe collectMarkerMotionData (
      MotionTargetController controller, ArrayList<FrameMarker> targets,
      MarkerMotionData motion, MarkerMapping map, double start, double stop) {
      NumericInputProbe targetProbe =
         InverseManager
            .createInputProbe (
               controller, ProbeID.TARGET_POSITIONS, null, start, stop);
      VectorNd positions = new VectorNd (3 * targets.size ());
      for (int i = 0; i < motion.numFrames (); i++) {
         for (int j = 0; j < targets.size (); j++) {
            String expName =
               map.getExpLabelFromModel (targets.get (j).getName ());
            Vector3d pos = motion.getMarkerPosition (i, expName);
            positions.set (3 * j, pos.x);
            positions.set (3 * j + 1, pos.y);
            positions.set (3 * j + 2, pos.z);
         }
         double time = motion.getFrameTime (i);
         targetProbe.addData (time, positions);
      }
      return targetProbe;
   }

   /**
    * Creates a {@link NumericControlProbe} for the specified body {@code frame}
    * and fills it with the force data in {@code forces}, based on the
    * identifier {@code side} (i.e. left or right).
    * 
    * @param forces
    * {@link ForceTarget}
    * @param frame
    * Rigid body to apply the input probe to
    * @param side
    * left or right
    * @param scale2 
    * @param stop 
    * 
    */
   private void createForceInputProbe (
      ForceData forces, Frame frame, String side, double start, double stop,
      double scale) {
      NumericControlProbe grf = new NumericControlProbe ();
      grf.setModel (myMech);
      grf.setName (frame.getName () + " ground reaction forces");
      grf.setStartStopTimes (start, stop);
      grf.setInterpolationOrder (Interpolation.Order.Linear);
      GroundReactionMomentFunction momentArm =
         new GroundReactionMomentFunction (frame, side, scale);
      GLViewer viewer = getMainViewer();
      if (viewer != null)
         viewer.addRenderable (momentArm);
      grf.setDataFunction (momentArm);
      grf.setVsize (6);
      for (int i = 0; i < forces.getFrame (stop); i++) {
         VectorNd force = new VectorNd (6);
         double time = forces.getFrameTime (i);
         force.add (0, forces.getData (i, side + " GRF").x);
         force.add (1, forces.getData (i, side + " GRF").y);
         force.add (2, forces.getData (i, side + " GRF").z);
         force.add (3, forces.getData (i, side + " GRM").x);
         force.add (4, forces.getData (i, side + " GRM").y);
         force.add (5, forces.getData (i, side + " GRM").z);
         grf.addData (time, force);
      }
      addInputProbe (grf);
   }

   /**
    * Creates and adds a {@link NumericMonitorProbe} with
    * {@link ExciterForceFunction} as data function to track the generated
    * forces for each exciter that is not a muscle. Created probes are smoothed
    * using a moving average filter whose windows size depends on the current
    * step size.
    * 
    * @param e
    * excitation component for which the probe is created
    * @param monitor 
    * @param start
    * start time
    * @param stop
    * stop tome
    * @param step
    * step size
    */
   private void createExciterOutputProbe (
      ExcitationComponent e, SmoothingMonitor monitor, double start,
      double stop, double step) {
      String probeName = e.getName () + "_force";
      String filepath =
         PathFinder
            .getSourceRelativePath (
               this, "/" + myName + "/Output/" + probeName + ".txt");
      NumericMonitorProbe momProbe =
         new NumericMonitorProbe (1, filepath, start, stop, step);
      momProbe.setModel (myMech);
      momProbe.setName (probeName);
      momProbe.setInterpolationOrder (Interpolation.Order.Linear);
      ExciterForceFunction force = new ExciterForceFunction (e);
      momProbe.setDataFunction (force);
      monitor.add (momProbe);
      addOutputProbe (momProbe);
   }

   /**
    * Generates an Output probe as {@link NumericDifferenceProbe}, that computes
    * the tracking error of every marker as difference between the positions in
    * {@code posProbe} (inverse kinematics positions) and {@code targetProbe}
    * (experimental marker trajectories).
    * 
    * @param targetProbe
    * IK positions
    * @param posProbe
    * IK Exp positions
    */
   private NumericDifferenceProbe createIKPositionError (
      NumericInputProbe targetProbe, PositionInputProbe posProbe) {
      NumericDifferenceProbe errProbe =
         new NumericDifferenceProbe (targetProbe, posProbe, getMaxStepSize ());
      errProbe.setName ("IK Position Error");
      errProbe.updateData (getMaxStepSize ());
      String path =
         PathFinder
            .getSourceRelativePath (
               this, myName + "/Output/IK position error.txt");
      errProbe.setAttachedFileName (path);
      errProbe.setActive (false);
      return errProbe;
   }

   /**
    * Generates achievable positions for the PointTargets in {@code targets} by
    * solving an inverse kinematics problem. The calculated new PointTarget
    * positions are then used as actual target positions by the
    * MotionTargetController.
    * 
    * @param targets
    * Collection of PointTargets
    * @param weights
    * VectorNd containing the marker weights
    * @param targetProbe
    * NumericInputProbe containing the original marker positions
    * @return
    */
   private PositionInputProbe createIKPositions (
      ArrayList<FrameMarker> targets, VectorNd weights,
      NumericInputProbe targetProbe) {
      String posProbeName = "IK target positions";
      IKSolver solver = new IKSolver (myMech, targets, weights);
      solver.setMaxIterations (100);
      PositionInputProbe posProbe =
         solver.createMarkerPositionProbe (posProbeName, targetProbe, true, -1);
      VectorNd targs0 = targetProbe.getNumericList ().getFirst ().v;
      int niters = solver.solve (targs0);
      if (myDebug)
         System.out.println ("in " + niters + " iterations");
      // Generate an IK probe for non dynamic visualization
      myIKProbe =
         new IKProbe (posProbeName, myMech, targets, weights,
            targetProbe.getStartTime (),
            targetProbe.getStopTime ());
      myIKProbe.setData (posProbe, true);
      addInputProbe (myIKProbe);
      myIKProbe.setActive (false);
      myIKProbe.setBodiesNonDynamicIfActive (true);
      return posProbe;
   }

   /**
    * Generates an output probe that calculates the normalized fiber length
    * during computation for the {@link ExcitationComponent} {@code e}.
    * 
    * @param e
    * ExcitationComponent
    * @param monitor 
    * @param cmcStart
    * start time
    * @param stop
    * stop time
    * @param step
    * step size
    */
   private void createMusFbrLgthOutputProbe (
      ExcitationComponent e, SmoothingMonitor monitor, double cmcStart,
      double stop, double step) {
      MultiPointMuscle muscle = (MultiPointMuscle)e;
      AxialMaterial mat = muscle.getMaterial ();
      String probeName = e.getName () + " normalized fiber length";
      String probePath =
         PathFinder
            .getSourceRelativePath (
               this, "/" + myName + "/Output/" + probeName + ".txt");
      NumericMonitorProbe fiberLength =
         new NumericMonitorProbe (1, probePath, cmcStart, stop, step);
      fiberLength.setModel (myMech);
      fiberLength.setName (probeName);
      fiberLength.setInterpolationOrder (Interpolation.Order.Linear);
      NormalizedFiberLengthFunction nFibLength =
         new NormalizedFiberLengthFunction (mat);
      fiberLength.setDataFunction (nFibLength);
      monitor.add (fiberLength);
      addOutputProbe (fiberLength);
   }

   /**
    * Creates a {@link NumericOutputProbe} for the given property {@code prop}
    * of the model component {@code comp} with the parameters {@code start},
    * {@code stop} and {@code step}. Adds a widget to a {@link ControlPanel} for
    * each {@code prop}, if {@code panel} is not null.
    * 
    * @param comp
    * Model component
    * @param panel
    * Control panel
    * @param prop
    * Name of the property
    * @param monitor
    * @param start
    * Start time of the probe
    * @param stop
    * Stop time of the probe
    * @param step
    * Output interval of the probe
    */
   private void createProbeAndPanel (
      ModelComponent comp, ControlPanel panel, String prop,
      SmoothingMonitor monitor, double start, double stop, double step) {
      String filepath =
         PathFinder
            .getSourceRelativePath (
               this, "/" + myName + "/Output/" + comp.getName () + " " + prop
               + ".txt");
      NumericOutputProbe probe =
         new NumericOutputProbe (comp, prop, filepath, step);
      if (panel != null) {
         String panelText = comp.getName () + " " + prop;
         panel.addWidget (panelText, comp, prop);
      }
      probe.setName (comp.getName () + " " + prop);
      probe.setStartStopTimes (start, stop);
      if (monitor != null) {
         monitor.add (probe);
      }
      addOutputProbe (probe);
   }

   /**
    * Creates a VelocityInputProbe to the according PositionInputProbe
    * {@code posProbe}.
    * 
    * @param posProbe
    * @return
    */
   private VelocityInputProbe createVelProbe (PositionInputProbe posProbe) {
      VelocityInputProbe velProbe =
         VelocityInputProbe
            .createInterpolated ("target velocities", posProbe, true, -1);
      // smooth velocities, since they were kind of wobbly. Done by John,
      // generalised by me
      int wsize = (int)(0.21 / getMaxStepSize ());
      velProbe.smoothWithSavitzkyGolay (wsize, 4);
      velProbe.setActive (true);
      System.out.println (
            "GaitModel: generated velocity input probes based on marker trajectories");
      return velProbe;
   }

   /**
    * Queries all {@link RigidBody} objects from the current {@link MechModel}
    * and stores them in a global variable.
    * 
    * @return list of rigid bodies
    */
   @SuppressWarnings("unchecked")
   private RenderableComponentList<RigidBody> getBodiesFromOsim () {
      myBodies = (RenderableComponentList<RigidBody>)myMech.get ("bodyset");
      // Set inertia mode to density is a body of zero mass is encountered
      myBodies.forEach (body -> {
         if (body.getMass () == 0) {
            System.out
               .println (
                  "GaitModel, warning: " + body.getName ()
                  + " has zero mass. Assigned density of 1 kg/m^3 to compute mass and moment of inertia.");
            body.setDensity (1);
            body.setInertiaMethod (InertiaMethod.DENSITY);
         }
      });
      return myBodies;
   }

   /**
    * Defines joint constraints based on jointsets or by defining the joints
    * based on the rigid bodies, that are part of the model.
    * 
    * @param myBodies
    * list of rigid bodies
    * @return list of {@link OpenSimCustomJoint}
    */
   private RenderableComponentList<JointBase> getJointsFromOsim (
      RenderableComponentList<RigidBody> bodies) {
      double comp = 0;
      RenderableComponentList<JointBase> joints;
      if (myMech.contains (myMech.get ("jointset"))) {
         System.out
            .println ("GaitModel: generated joint constraints from jointset.");
         joints = getJointsFromJointset (comp);
      }
      else {
         System.out
            .println (
               "GaitModel: generated joint constraints from ridig body connectors.");
         joints = getJointsFromBodyset (bodies, comp);
      }
      joints.forEach (joint -> {
         switch (joint.getName ()) {
            case "subtalar_l":
            case "subtalar_r":
            case "mtp_l":
            case "mtp_r":
            case "radius_hand_r":
            case "radius_hand_l":
            case "r_scapulothoracic":
            case "l_scapulothoracic":
            case "r_sternoclavicular":
            case "l_sternoclavicular":
               for (int i = 0; i < joint.numCoordinates (); i++) {
                  joint.setCoordinateLocked (i, true);
               }
               break;
         }
      });
      return joints;
   }

   /**
    * Defines joints by accessing the connectors of every body and writing them
    * to a separate list. Connectors are defined per rigid body, a body that is
    * connected to 3 bodies, has therefore three different connectors. The
    * connectors in each rigid body are ordered in such a way, that each joint
    * is exactly addressed once, if the first index of the connector list
    * (getConnectors.get(0)) in every rigid body is accessed.
    * 
    * @param bodies 
    * @param compMagnitude
    * compliance magnitude
    * @return list of {@link OpenSimCustomJoint}
    */
   private RenderableComponentList<JointBase> getJointsFromBodyset (
      RenderableComponentList<RigidBody> bodies, double compMagnitude) {
      // Ground shares no joint with any rigid body else than the hip
      // so skip that, since hip is going to be addressed either way.
      bodies.forEach (rb -> {
         if (rb.getName ().equals ("ground")) 
            return;
         else {
            // Write all joints to a joint list
            myJoints.add ((JointBase)rb.getConnectors ().get (0));
            int end = myJoints.size ();
            setJointCompliance (myJoints.get (end), compMagnitude);
            DoubleInterval range = new DoubleInterval ();
            switch (rb.getName ()) {
               // specify the joint constraints for each joint individually
               // by addressing the respective dof (int idx) and its range.
               case "pelvis":
                  range.set (-90, 90);
                  myJoints.get (end - 1).setCoordinateRangeDeg (0, range);
                  myJoints.get (end - 1).setCoordinateRangeDeg (1, range);
                  myJoints.get (end - 1).setCoordinateRangeDeg (2, range);
                  range.set (-5, 5);
                  myJoints.get (end - 1).setCoordinateRange (3, range);
                  range.set (-1, 2);
                  myJoints.get (end - 1).setCoordinateRange (4, range);
                  range.set (-3, 3);
                  myJoints.get (end - 1).setCoordinateRange (5, range);
                  break;
               case "femur_r":
                  range.set (-120.0, 120.0);
                  myJoints.get (end - 1).setCoordinateRangeDeg (0, range);
                  myJoints.get (end - 1).setCoordinateRangeDeg (1, range);
                  myJoints.get (end - 1).setCoordinateRangeDeg (2, range);
                  break;
               case "tibia_r":
                  range.set (-120.0, 10);
                  myJoints.get (end - 1).setCoordinateRangeDeg (0, range);
                  break;
               case "talus_r":
                  range.set (-90, 90);
                  myJoints.get (end - 1).setCoordinateRangeDeg (0, range);
                  break;
               case "calcn_r":
                  range.set (0, 0);
                  myJoints.get (end - 1).setCoordinateRangeDeg (0, range);
                  break;
               case "toes_r":
                  range.set (0, 0);
                  myJoints.get (end - 1).setCoordinateRangeDeg (0, range);
                  break;
               case "femur_l":
                  range.set (-120.0, 120.0);
                  myJoints.get (end - 1).setCoordinateRangeDeg (0, range);
                  myJoints.get (end - 1).setCoordinateRangeDeg (1, range);
                  myJoints.get (end - 1).setCoordinateRangeDeg (2, range);
                  break;
               case "tibia_l":
                  range.set (-120.0, 10);
                  myJoints.get (end - 1).setCoordinateRangeDeg (0, range);
                  break;
               case "talus_l":
                  range.set (-90, 90);
                  myJoints.get (end - 1).setCoordinateRangeDeg (0, range);
                  break;
               case "calcn_l":
                  range.set (0, 0);
                  myJoints.get (end - 1).setCoordinateRangeDeg (0, range);
                  break;
               case "toes_l":
                  range.set (0, 0);
                  myJoints.get (end - 1).setCoordinateRangeDeg (0, range);
                  break;
               case "torso":
                  range.set (-90, 90);
                  myJoints.get (end - 1).setCoordinateRangeDeg (0, range);
                  myJoints.get (end - 1).setCoordinateRangeDeg (1, range);
                  myJoints.get (end - 1).setCoordinateRangeDeg (2, range);
                  break;
               case "acromial_r":
                  range.set (-90, 90);
                  myJoints.get (end - 1).setCoordinateRangeDeg (0, range);
                  range.set (-120, 90);
                  myJoints.get (end - 1).setCoordinateRangeDeg (1, range);
                  range.set (-90, 90);
                  myJoints.get (end - 1).setCoordinateRangeDeg (2, range);
                  break;
               case "elbow_r":
                  range.set (0, 150);
                  myJoints.get (end - 1).setCoordinateRangeDeg (0, range);
                  break;
               case "radioulnar_r":
                  range.set (0, 150);
                  myJoints.get (end - 1).setCoordinateRangeDeg (0, range);
                  break;
               case "radius_hand_r":
                  range.set (-70, 70);
                  myJoints.get (end - 1).setCoordinateRangeDeg (0, range);
                  range.set (-25, 35);
                  myJoints.get (end - 1).setCoordinateRangeDeg (1, range);
                  break;
               case "acromial_l":
                  range.set (-90, 90);
                  myJoints.get (end - 1).setCoordinateRangeDeg (0, range);
                  range.set (-120, 90);
                  myJoints.get (end - 1).setCoordinateRangeDeg (1, range);
                  range.set (-90, 90);
                  myJoints.get (end - 1).setCoordinateRangeDeg (2, range);
                  break;
               case "elbow_l":
                  range.set (0, 150);
                  myJoints.get (end - 1).setCoordinateRangeDeg (0, range);
                  break;
               case "radioulnar_l":
                  range.set (0, 150);
                  myJoints.get (end - 1).setCoordinateRangeDeg (0, range);
                  break;
               case "radius_hand_l":
                  range.set (-70, 70);
                  myJoints.get (end - 1).setCoordinateRangeDeg (0, range);
                  range.set (-25, 35);
                  myJoints.get (end - 1).setCoordinateRangeDeg (1, range);
                  break;
               case "r_scapulothoracic":
                  range.set (-70, 70);
                  myJoints.get (end - 1).setCoordinateRangeDeg (0, range);
                  break;
               case "r_sternoclavicular":
                  range.set (-70, 70);
                  myJoints.get (end - 1).setCoordinateRangeDeg (0, range);
                  break;
               case "l_scapulothoracic":
                  range.set (-70, 70);
                  myJoints.get (end - 1).setCoordinateRangeDeg (0, range);
                  break;
               case "l_sternoclavicular":
                  range.set (-70, 70);
                  myJoints.get (end - 1).setCoordinateRangeDeg (0, range);
                  break;
            }
         }
      });
      return myJoints;
   }

   /**
    * Defines joints by accessing the jointsets predefined by the .osim file.
    * 
    * @param compMagnitude
    * compliance magnitude
    * @return list of {@link OpenSimCustomJoint}
    */
   @SuppressWarnings("unchecked")
   private RenderableComponentList<JointBase> getJointsFromJointset (
      double compMagnitude) {
      myJoints = (RenderableComponentList<JointBase>)myMech.get ("jointset");
      DoubleInterval range = new DoubleInterval ();
      myJoints.forEach (jt -> {
         setJointCompliance (jt, compMagnitude);
         // Define joint limits for each joint constraint
         switch (jt.getName ()) {
            case "ground_pelvis":
               range.set (-180, 180);
               jt.setCoordinateRangeDeg (0, range);
               jt.setCoordinateRangeDeg (1, range);
               jt.setCoordinateRangeDeg (2, range);
               range.set (-5, 5);
               jt.setCoordinateRange (3, range);
               range.set (-5, 5);
               jt.setCoordinateRange (4, range);
               range.set (-5, 5);
               jt.setCoordinateRange (5, range);
               break;
            case "hip_r":
               range.set (-120.0, 120.0);
               jt.setCoordinateRangeDeg (0, range);
               jt.setCoordinateRangeDeg (1, range);
               jt.setCoordinateRangeDeg (2, range);
               break;
            case "knee_r":
               range.set (-120.0, 10);
               jt.setCoordinateRangeDeg (0, range);
               break;
            case "ankle_r":
               range.set (-90, 90);
               jt.setCoordinateRangeDeg (0, range);
               break;
            case "subtalar_r":
               range.set (0, 0);
               jt.setCoordinateRangeDeg (0, range);
               break;
            case "mtp_r":
               range.set (0, 0);
               jt.setCoordinateRangeDeg (0, range);
               break;
            case "hip_l":
               range.set (-120.0, 120.0);
               jt.setCoordinateRangeDeg (0, range);
               jt.setCoordinateRangeDeg (1, range);
               jt.setCoordinateRangeDeg (2, range);
               break;
            case "knee_l":
               range.set (-120.0, 10);
               jt.setCoordinateRangeDeg (0, range);
               break;
            case "ankle_l":
               range.set (-90, 90);
               jt.setCoordinateRangeDeg (0, range);
               break;
            case "subtalar_l":
               range.set (0, 0);
               jt.setCoordinateRangeDeg (0, range);
               break;
            case "mtp_l":
               range.set (0, 0);
               jt.setCoordinateRangeDeg (0, range);
               break;
            case "back":
               range.set (-90, 90);
               jt.setCoordinateRangeDeg (0, range);
               jt.setCoordinateRangeDeg (1, range);
               jt.setCoordinateRangeDeg (2, range);
               break;
            case "acromial_r":
               range.set (-90, 90);
               jt.setCoordinateRangeDeg (0, range);
               range.set (-120, 90);
               jt.setCoordinateRangeDeg (1, range);
               range.set (-90, 90);
               jt.setCoordinateRangeDeg (2, range);
               break;
            case "elbow_r":
               range.set (0, 150);
               jt.setCoordinateRangeDeg (0, range);
               break;
            case "radioulnar_r":
               range.set (0, 150);
               jt.setCoordinateRangeDeg (0, range);
               break;
            case "radius_hand_r":
               range.set (-70, 70);
               jt.setCoordinateRangeDeg (0, range);
               range.set (-25, 35);
               jt.setCoordinateRangeDeg (1, range);
               break;
            case "acromial_l":
               range.set (-90, 90);
               jt.setCoordinateRangeDeg (0, range);
               range.set (-120, 90);
               jt.setCoordinateRangeDeg (1, range);
               range.set (-90, 90);
               jt.setCoordinateRangeDeg (2, range);
               break;
            case "elbow_l":
               range.set (0, 150);
               jt.setCoordinateRangeDeg (0, range);
               break;
            case "radioulnar_l":
               range.set (0, 150);
               jt.setCoordinateRangeDeg (0, range);
               break;
            case "radius_hand_l":
               range.set (-70, 70);
               jt.setCoordinateRangeDeg (0, range);
               range.set (-25, 35);
               jt.setCoordinateRangeDeg (1, range);
               break;
            case "r_scapulothoracic":
               range.set (-70, 70);
               jt.setCoordinateRangeDeg (0, range);
               break;
            case "r_sternoclavicular":
               range.set (-70, 70);
               jt.setCoordinateRangeDeg (0, range);
               break;
            case "l_scapulothoracic":
               range.set (-70, 70);
               jt.setCoordinateRangeDeg (0, range);
               break;
            case "l_sternoclavicular":
               range.set (-70, 70);
               jt.setCoordinateRangeDeg (0, range);
               break;
         }
      });
      return myJoints;
   }

   /**
    * Queries all {@link FrameMarker} objects from the current {@link MechModel}
    * and stores them in a global variable.
    * 
    * @return list of frame markers
    */
   @SuppressWarnings("unchecked")
   private RenderableComponentList<FrameMarker> getMarkerFromOsim () {
      myMarkers =
         (RenderableComponentList<FrameMarker>)myMech.get ("markerset");
      System.out.println ("GaitModel: model markers: " + myMarkers.size ());
      // Overwrite attachments for toe markers, since attached to calcanei
      myMarkers.forEach (m -> {
         RigidBody newFrame = null;
         RigidBody oldFrame = null;
         Vector3d newRef;
         Vector3d pos;
         if (m.getName ().contains ("R_Toe")) {
            newFrame = myBodies.get ("toes_r");
            oldFrame = (RigidBody)m.getFrame ();
         }
         else if (m.getName ().contains ("L_Toe")) {
            newFrame = myBodies.get ("toes_l");
            oldFrame = (RigidBody)m.getFrame ();
         }
         else
            return;
         Point3d newLoc = new Point3d (0, 0, 0);
         newRef = (Vector3d)newFrame.getPosition ();
         pos = (Vector3d)m.getPosition ();
         newLoc.sub (pos, newRef);
         // Set new attachment
         m.setFrame (newFrame);
         // Overwrite refpos vector after setting attachment
         m.setRefPos ((Point3d)newRef);
         // Overwrite location vector after setting attachment
         m.setLocation (newLoc);
         System.out
            .println (
               "GaitModel, warning: Attachment adjusted for " + m.getName () + " from "
               + oldFrame.getName () + " to " + newFrame.getName ());
      });
      return myMarkers;
   }

   /**
    * Returns an ArrayList of {@link MuscleComponent} objects for the muscle
    * identifiers contained in {@code names}.
    * 
    * @param names
    * String containing muscle names
    * @return
    */
   private ArrayList<MuscleComponent> getMuscles (String[] names) {
      HashSet<String> muscleNames = new HashSet<> ();
      for (String name : names) {
         muscleNames.add (name);
      }
      ArrayList<MuscleComponent> muscles = new ArrayList<> ();
      for (MuscleComponent msc : myMuscles) {
         if (muscleNames.contains (msc.getName ()))
            muscles.add (msc);
      }
      return muscles;
   }

   /**
    * Queries all {@link MuscleComponent} objects from the current
    * {@link MechModel} and stores them in a global variable.
    * 
    * @return list of multi point muscles
    */
   @SuppressWarnings("unchecked")
   private List<MuscleComponent> getMusclesFromOsim () {
      // Make use of MuscleComponents interface
      ForceSpringBase.useMuscleComponents = true;
      RenderableComponentList<ModelComponent> forces =
         (RenderableComponentList<ModelComponent>)myMech.get ("forceset");
      forces.forEach (frc -> {
         if (!(frc instanceof MuscleComponent))
            return;
         MuscleComponent muscle = (MuscleComponent)frc;
         muscle.setExcitation (0.0);
         myMuscles.add (muscle);
      });
      return myMuscles;
   }

   /**
    * Returns the name of the current working directory folder.
    * 
    * @param fc
    * @return
    */
   private String getNameFromFileDiaglog (JFileChooser fc) {
      fc.setCurrentDirectory (ArtisynthPath.getSrcRelativeFile (this, myName));
      fc.setFileSelectionMode (JFileChooser.DIRECTORIES_ONLY);
      fc.setDialogTitle ("Please select a working directory.");
      fc.showOpenDialog (null);
      return fc.getSelectedFile ().getName ();
   }

   /**
    * Defines all in- and outgoing probes for the model. Ingoing probes can be
    * experimental marker trajectories, forces or generalized coordinates,
    * outgoing probes can be all joint angles (if not already specified as
    * generalized coordinates), muscle and frame exciter excitations.
    * 
    * @param name
    * Name specifier of the current working directory
    * @param scale
    * Unit scaling factor for the current model (m = 1, mm = 1000)
    * @throws IOException
    * if specified files or file paths are invalid
    * @throws CloneNotSupportedException 
    */
   private void initializeIOProbes (String name, double scale)
      throws IOException {
      // Read all input data
      myMap = readMarkerFile (name);
      myMotion = readTRCFile (name, scale, myMap);
      double start = 0.0;
      double stop = myMotion.getFrameTime (myMotion.numFrames () - 1);
      myForces = readForceFile (name);
      myCoords = readCoordsFile (name);
      // Setup controller
      if (myUseCAsSolely) {
         ComponentUtils.deleteComponentsAndDependencies (myMuscles);
         myMuscles.removeAll (myMuscles);
      }      
      MotionTargetController controller = addControllerAndProps ();
      addExcitersToController (controller);
      addPointTargetsAndProbes (controller, myMap, myMotion, start, stop);
      addForceInputProbes (myForces, start, stop, scale);
      // Add output probes
      if (myUseIKSolely) {
         setupInverseKinematicsRun ();
      }
      addNumOutputProbesAndPanel (myMotion, controller, start, stop);
      // Stop simulation after last frame
      addBreakPoint (stop);
   }

   /**
    * Imports an OpenSim model from the current working directory, defines joint
    * constraints and stores important model components in variables.
    * 
    * @param myName
    * Name specifier of the current working directory
    * @param scale
    * Unit scale factor for the current model (m = 1, mm = 1000)
    * @throws IOException
    */
   private void initializeOsim (String myName, double scale)
      throws IOException {
      readOsimFile (myName, scale); 
      myBodies = getBodiesFromOsim ();
      myJoints = getJointsFromOsim (myBodies);
      myMuscles = getMusclesFromOsim ();
      myMarkers = getMarkerFromOsim ();
      myMech.scaleDistance (scale);
      myMech.scaleMass (scale);
   }

   /**
    * Returns joint angles as generalized coordinates from prior IK calculations
    * or MoCap systems ({@code name_angles.mot}) in the input folder of the
    * current working directory.
    * 
    * @param name
    * Name specifier of the current working directory
    * @return {@link CoordinateData} object
    * @throws IOException
    * if the file or file path are invalid
    */
   private CoordinateData readCoordsFile (String name) throws IOException {
      String motName = name + "/Input/" + name + "_angles.mot";
      File motFile = ArtisynthPath.getSrcRelativeFile (this, motName);
      if (!motFile.exists () || motFile.isDirectory ())
         return null;
      MOTReader motReader = new MOTReader (motFile);
      motReader.readData ();
      // Print reading details to the console
      System.out
         .println (
            "Calculated generalized coordinates: "
            + motReader.getNumCoordLabels ());
      System.out
         .println (
            "MOT file: read " + motReader.getNumCoordFrames () + " frames");
      return motReader.getCoordinateData ();
   }

   /**
    * Returns ground reaction forces from the available experimental force data
    * ({@code name_forces.mot}) in the input folder of the current working
    * directory.
    * 
    * @param name
    * Name specifier of the current working directory
    * @return {@link ForceData} object
    * @throws IOException
    * if the file or file path are invalid
    */
   private ForceData readForceFile (String name) throws IOException {
      String motName = name + "/Input/" + name + "_forces.mot";
      File motFile = ArtisynthPath.getSrcRelativeFile (this, motName);
      if (!motFile.exists () || motFile.isDirectory ())
         return null;
      // Specify plate and movement direction upon first contact
      List<Object> forceSetupFlags = readForceSetupFile (name);
      String side = (String)forceSetupFlags.get (0);
      int num = (int)forceSetupFlags.get (1);
      MOTReader motReader = new MOTReader (motFile, side, num);
      motReader.readData ();
      return motReader.getForceData ();
   }

   /**
    * Reads the keywords in {@code name_setup.txt} in the input folder of the
    * current working directory to determine the body side and force plate
    * number associated with the first contact event. Experimental ground
    * reaction forces from that force plate will be assigned to that body side
    * later on.
    * 
    * @param name
    * Name specifier of the current working directory
    * @return Body side (left or right) and force plate no (1 or 2) of the first
    * contact event
    * @throws IOException
    * if the file or file path are invalid
    */
   private List<Object> readForceSetupFile (String name) throws IOException {
      String setupName = name + "/Input/" + name + "_setup.txt";
      File setupFile = ArtisynthPath.getSrcRelativeFile (this, setupName);
      if (!setupFile.exists () || setupFile.isDirectory ())
         return null;
      List<Object> flags = new ArrayList<Object> ();
      try (
      BufferedReader reader = new BufferedReader (new FileReader (setupFile))) {
         String line;
         // Skip the first line (header)
         reader.readLine ();
         while ((line = reader.readLine ()) != null) {
            line = line.trim ();
            String[] tks = line.split ("\t");
            if (tks.length != 2)
               throw new IOException (
                  "Error: keyword and value expected in " + name
                  + "_setup.txt");
            if (tks[0] == "name" && tks[1] != name) {
               throw new IOException (
                  "Error: name in " + name + "_setup.txt != " + name);
            }
            if (tks[0].equals ("side of 1st contact"))
               flags.add (tks[1]);
            if (tks[0].equals ("plate# of 1st contact"))
               flags.add (Integer.parseInt (tks[1]));
         }
      }
      catch (NumberFormatException e) {
         e.printStackTrace ();
      }
      if (flags.size () == 0)
         throw new IOException (
            "Error: side and plate# of 1st contact not contained in " + name
            + "_setup.txt.");
      return flags;
   }

   /**
    * Returns a map of marker names based on an input file. It accepts files in
    * a specific format: No head line, all lines divided by a tab. 1st column:
    * Name of the model marker 2nd column: Name of the experimental marker 3rd
    * Column: Marker weighting for the inverse solver If no experimental marker
    * exists for a given model marker, leave out the 2nd and 3rd column and
    * continue with next line.
    * 
    * @param name
    * Name specifier of the model working directory
    * @return {@link MarkerMapping} object
    * @throws IOException
    * if the file or file path are invalid
    */
   private MarkerMapping readMarkerFile (String name) throws IOException {
      String mapName = name + "/Input/" + name + "_markers.txt";
      File mapFile = ArtisynthPath.getSrcRelativeFile (this, mapName);
      if (!mapFile.exists () || mapFile.isDirectory ())
         return null;
      BufferedReader reader = new BufferedReader (new FileReader (mapFile));
      ArrayList<String> modelLabels = new ArrayList<String> ();
      ArrayList<String> expLabels = new ArrayList<String> ();
      ArrayList<Double> weights = new ArrayList<Double> ();
      String line;
      // Skip first line (header)
      reader.readLine ();
      while ((line = reader.readLine ()) != null) {
         line = line.trim ();
         assert line.length () != 0;
         String[] markers = line.split ("\t");
         double wght = Double.parseDouble (markers[2]);
         if (wght == 0) {
            System.out
               .println (
                  "GaitModel, warning: Marker " + markers[0]
                  + ": No corresponding experimental marker detected.");
            modelLabels.add (markers[0]);
            expLabels.add (null);
            weights.add (null);
         }
         else {
            modelLabels.add (markers[0]);
            expLabels.add (markers[1]);
            weights.add (wght);
         }
      }
      reader.close ();
      return new MarkerMapping (modelLabels, expLabels, weights);
   }

   /**
    * Reads in the .osim file in the specified working folder and creates a
    * corresponding model the current {@link MechModel}.
    * 
    * @param name
    * Name specifier for the current working directory
    * @param scale
    * Scaling factor for the current model (m = 1, mm = 1000)
    */
   private void readOsimFile (String name, double scale) {
      String modelPath = myName + "/" + myName + "_scaled.osim";
      File osimFile = ArtisynthPath.getSrcRelativeFile (this, modelPath);
      String geometryPath = myName + "/Geometry/";
      File geometryFile = ArtisynthPath.getSrcRelativeFile (this, geometryPath);
      if (osimFile.exists () && geometryFile.exists ()) {
         OpenSimParser parser = new OpenSimParser (osimFile, geometryFile);
         parser.createModel (myMech);
      }
   }

   /**
    * Returns marker trajectories from the available experimental motion data
    * ({@code name_positions.trc}) in the input folder of the current working
    * directory.
    * 
    * @param name
    * Name specifier of the current working directory
    * @param scale
    * Unit scaling factor for the current model (m = 1, mm = 1000)
    * @param map
    * model and experimental marker names and weights
    * @return {@link MarkerMotionData} object
    * @throws IOException
    * if the file or file path are invalid
    */
   private MarkerMotionData readTRCFile (
      String name, double scale, MarkerMapping map)
      throws IOException {
      String trcName = name + "/Input/" + name + "_positions.trc";
      File trcFile = ArtisynthPath.getSrcRelativeFile (this, trcName);
      if (!trcFile.exists () || trcFile.isDirectory ()) 
         return null;
      CustomTRCReader trcReader = new CustomTRCReader (trcFile, map);
      trcReader.readData ();
      MarkerMotionData motion = trcReader.getMotionData ();
      // Scale the marker trajectories individually, since there is no
      // general .scale () method for marker positions if the unit is not
      // already mm
      if (trcReader.getUnits ().equals ("mm")) {
         for (int i = 0; i <= motion.numFrames () - 1; i++) {
            motion.getMarkerPositions (i).forEach (p -> {
               Vector3d buf = new Vector3d (p.x, p.y, p.z);
               p.x = buf.x * scale / 1000;
               p.y = buf.y * scale / 1000;
               p.z = buf.z * scale / 1000;
            });
         }
      }
      return motion;
   }

   /**
    * Defines global and individual contact properties of the entire model.
    * Individual contact is enforced by defining a collision behavior object for
    * each interface (joints, ground contact, etc.), that contains compliant
    * contact properties. To monitor contact events, a collision response object
    * is created for each interface vice versa. The list of collision responses
    * is handed to the contact monitor class object.
    * 
    * @param coll
    * {@link CollisionManager}
    * @param name
    * working directory identifier
    * @throws IOException
    */
   private void setContactProps (CollisionManager coll, String name)
      throws IOException {
      coll.setName ("Collision manager");
      // Handle overconstrained contact
      coll.setReduceConstraints (true);
      coll.setBilateralVertexContact (false);
      // Smooth vertex penetration contatcs
      coll.getSmoothVertexContacts ();
   }

   /**
    * Initializes the default output probes of the inverse controller (tracked
    * positions, source positions and computed excitations) and adjusts the save
    * path.
    * 
    * @param controller
    * {@link MotionTargetController}
    * @param start
    * start time
    * @param stop
    * stop time
    * 
    */
   private void setDefaultOutputProbes (
      MotionTargetController controller, double start, double stop) {
      String path =
         PathFinder
            .getSourceRelativePath (
               this, myName + "/Output/tracked positions.txt");
      NumericOutputProbe trackedPos =
         InverseManager
            .createOutputProbe (
               controller, ProbeID.TRACKED_POSITIONS, path, start, stop, -1);
      path =
         PathFinder
            .getSourceRelativePath (
               this, myName + "/Output/source positions.txt");
      NumericOutputProbe sourcePos =
         InverseManager
            .createOutputProbe (
               controller, ProbeID.SOURCE_POSITIONS, path, start, stop, -1);
      path =
         PathFinder
            .getSourceRelativePath (
               this, myName + "/Output/computed excitations.txt");
      NumericOutputProbe compExc =
         InverseManager
            .createOutputProbe (
               controller, ProbeID.COMPUTED_EXCITATIONS, path, start, stop, -1);
      addOutputProbe (trackedPos);
      addOutputProbe (sourcePos);
      addOutputProbe (compExc);
   }

   /**
    * Define compliance for the provided joint to prevent overconstraints.
    * 
    * @param jt
    * {@link JointBase} object
    * @param compMagnitude
    * compliance magnitude
    */
   private void setJointCompliance (JointBase jt, double compMagnitude) {
      VectorNd comp = new VectorNd (jt.numConstraints ());
      VectorNd damp = new VectorNd (jt.numConstraints ());
      for (int i = 0; i < jt.numConstraints (); i++) {
         comp.set (i, compMagnitude);
         Frame bodyA = (Frame)jt.getBodyA ();
         Frame bodyB = (Frame)jt.getBodyB ();
         double mass = bodyA.getEffectiveMass () + bodyB.getEffectiveMass ();
         damp.set (i, 2 * 1 * Math.sqrt (mass / comp.get (i)));
      }
      jt.setCompliance (comp);
      jt.setDamping (damp);
   }

   /**
    * Sets the following simulation properties: 1. Solver, 2. Step sizes, 3.
    * global damping parameters, 4. model unit scaling and 6. gravity
    */
   private void setSimulationProperties () {
      // Solver properties
      MechSystemSolver.setHybridSolvesEnabled (false);
      MechSystemSolver solver = myMech.getSolver ();
      // if not coded separately, the hybrid solves prop is wrong in the
      // input.txt
      solver.setHybridSolve (false);
      solver.setIntegrator (Integrator.FullBackwardEuler);
      solver.setStabilization (PosStabilization.GlobalStiffness);
      setMaxStepSize (1e-3);
      // mech model properties
      myMech.setInertialDamping (1.0);
      // Collect constraint forces in the "force" fields of particles and bodies
      myMech.setUpdateForcesAtStepEnd (true);
      myMech.setGravity (new Vector3d (0, -9.81, 0));
      if (myMech.getGravity ().equals (new Vector3d (0, 0, 0))) {
         JFrame frame = new JFrame ("Warning");
         frame.add (new JLabel ("GaitModel, warning: Zero gravitation!", JLabel.CENTER));
         frame.setSize (300, 100);
         frame.setLocationRelativeTo (getMainFrame ());
         // Visibility always at last
         frame.setVisible (true);
      }
   }

   /**
    * Enables the "Idealised CMC" run, that relies on one CoordinateActuator per
    * Joint DoF. Resets the controller settings to the GaitModel defaults.
    * 
    * @throws IOException
    */
   private void setupCoordinateActuatorRun () throws IOException {
      // Delete muscles from system and thereby also from the controller
      ComponentUtils.deleteComponentsAndDependencies (myMuscles);
      myMuscles.removeAll (myMuscles);
      // Reset controller parameters to GaitModel "defaults"
      MotionTargetController controller =
         (MotionTargetController)getControllers ().get ("Motion controller");
      controller.setMotionTargetAsConstraint (false);
      controller.removeExcitationDamping ();
      controller.setL2Regularization ();
      MotionTargetTerm mTerm = controller.getMotionTargetTerm ();
      mTerm.setChaseTime (0.001);
      mTerm.setKd (1e-1 / getMaxStepSize ());
      mTerm.setKp (1 / (getMaxStepSize () * mTerm.getChaseTime ()));
      // Reset exciter weights all to one
      controller.getExciters ().forEach (ex -> {
         controller.setExcitationWeight (ex, 1);
      });

      writeInputToFile (myName);
   }

   /**
    * Enables Inverse Kinematics for the currently active model by setting
    * dynamics and the current tracking controller instance off. Also
    * deactivates all InputProbes except the IK target positions.
    * 
    * @throws IOException
    */
   private void setupInverseKinematicsRun () throws IOException {
      myMech.setDynamicsEnabled (false);
      MotionTargetController controller =
         (MotionTargetController)getControllers ().get ("Motion controller");
      controller.setActive (false);
      ComponentList<Probe> iProbes = getInputProbes ();
      iProbes.forEach (probe -> {
         if (probe.getName ().contains ("IK target positions"))
            probe.setActive (true);
         else
            probe.setActive (false);
      });

      writeInputToFile (myName);
   }

   /**
    * Adds contact information to the output string.
    * 
    * @param output
    * {@link StringBuilder} string
    * @param coll
    * {@link CollisionManager} of the current {@link MechModel}
    * @param behav
    * list of {@link CollisionBehaviorList} objects
    */
   private void writeContactInfo (
      StringBuilder output, CollisionManager coll,
      CollisionBehaviorList behav) {
      output
         .append ("\n%%CONTACT%%\n")
         .append (
            "%%------------------------------------------------------------%%\n")
         .append ("\nGLOBAL\n").append ("Contact Method: ")
         .append (coll.getMethod ().toString ()).append ("\n")
         .append ("Contact Region Detection Algorithm (Collision Type): ")
         .append (coll.getColliderType ().toString ()).append ("\n")
         .append ("Reduce overconstrained contact: ")
         .append (coll.getReduceConstraints ()).append ("\n")
         .append ("Number of contact interfaces: ")
         .append (coll.behaviors ().size ()).append ("\n")
         .append ("\nINDIVIDUAL\n");
      // Access each individual contact behavior for each model component
      behav.forEach (cb -> {
         output
            .append ("Contact Interface: ").append (cb.getName ()).append ("\n")
            .append ("Bilateral vertex contact: ")
            .append (cb.getBilateralVertexContact ()).append ("\n")
            .append ("Body or Group A: ")
            .append (cb.getCollidablePair ().get (0).getName ()).append ("\t")
            .append ("Body or Group B: ")
            .append (cb.getCollidablePair ().get (1).getName ()).append ("\n")
            .append ("Penetration Tolerance: ")
            .append (String.format ("%.3f", cb.getPenetrationTol ()))
            .append ("\n").append ("Contact compliance: ")
            .append (String.format ("%.3f", cb.getCompliance ())).append ("\n")
            .append ("Contact damping: ")
            .append (String.format ("%.3f", cb.getDamping ())).append ("\n\n");
      });
   }

   /**
    * Adds FEM information to the output string.
    * 
    * @param output
    * {@link StringBuilder} string
    * @param meshes
    * list of {@link FemModel3d} objects
    */
   private void writeFEMInfo (StringBuilder output, List<FemModel3d> meshes) {
      output
         .append ("%%FINITE ELEMENTS%%\n").append (
            "%%------------------------------------------------------------%%\n");
      output
         .append ("\nNumber of Meshes: ").append (myMeshes.size ())
         .append ("\n\n");
      myMeshes.forEach (mesh -> {
         output
            .append (mesh.getName ()).append ("\t")
            .append (String.format ("%.3f", mesh.getMass ())).append ("\tkg\n")
            .append ("Number of Nodes: ").append (mesh.numNodes ())
            .append ("\n").append ("Number of Elements: ")
            .append (mesh.numAllElements ()).append ("\n").append ("Density: ")
            .append (mesh.getDensity ()).append ("\tkg/m^3\n");
         LinearMaterial material = (LinearMaterial)mesh.getMaterial ();
         output
            .append ("Young's modulus: ").append ("\t")
            .append (material.getYoungsModulus ()).append ("\tN/m^2\n")
            .append ("Poisson's ratio: ").append (material.getPoissonsRatio ())
            .append ("\n");
         PointList<FemNode3d> nodes = mesh.getNodes ();
         nodes.forEach (n -> {
            output
               .append (n.getNumber ()).append ("\t")
               .append (n.getPosition ().toString ("%.3f")).append ("\n");
         });
         ArrayList<FemElement3dBase> elements = mesh.getAllElements ();
         elements.forEach (elem -> {
            output.append (elem.getNumber ()).append ("\t");
            FemNode3d[] elemNodes = elem.getNodes ();
            for (FemNode3d en : elemNodes) {
               output.append (en.getNumber ()).append ("\t");
            }
            output.append ("\n");
         });
         output.append ("\n");
      });
   }

   /**
    * Adds joint information to the output string.
    * 
    * @param output
    * {@link StringBuilder} string
    * @param joints
    * list of {@link JointBase} objects
    */
   private void writeJointInfo (
      StringBuilder output, RenderableComponentList<JointBase> joints) {
      if (joints == null)
         return;
      output
         .append ("JOINTS\n").append ("Number of joints: ")
         .append (joints.size ()).append ("\n");
      String format = "%d DOF %-20s%-14s\tType: %-6s%n";
      joints.forEach (jt -> {
         output
            .append ("Name: ").append (jt.getName ()).append ("\t")
            .append ("Body A: ").append (jt.getBodyA ().getName ())
            .append ("\t").append ("Body B: ")
            .append (jt.getBodyB ().getName ()).append ("\n")
            .append ("Compliance: ").append (jt.getCompliance ().toString ())
            .append ("\n").append ("Damping: ")
            .append (jt.getDamping ().toString ("%.3f")).append ("\n");
         for (int i = 0; i < jt.numCoordinates (); i++) {
            double min = jt.getCoordinateRangeDeg (i).getLowerBound ();
            double max = jt.getCoordinateRangeDeg (i).getUpperBound ();
            output
               .append (
                  String
                     .format (
                        format, i, jt.getCoordinateName (i),
                        String.format ("[%.1f, %.1f]", min, max),
                        jt.getCoordinateMotionType (i).toString ()));
         }
         output.append ("\n");
      });
   }

   /**
    * Adds rigid body information to the output string.
    * 
    * @param output
    * {@link StringBuilder} string
    * @param path
    * Path to the used .osim file
    * @param bodies
    * list of {@link RidigBody} objects
    */
   private void writeModelInfo (
      StringBuilder output, String path,
      RenderableComponentList<RigidBody> bodies) {
      if (bodies == null)
         return;
      output
         .append ("\n%%MODELBASE%%\n")
         .append (
            "%%------------------------------------------------------------%%\n")
         .append ("\nBODIES\n").append ("Model: ").append (path).append ("\n")
         .append ("Number of bodies: ").append (bodies.size ()).append ("\n")
         .append ("Total model mass: ")
         .append (String.format ("%.1f", myMech.getActiveMass ()))
         .append (" kg\n\n");;
      // append individual info for each body
      bodies.forEach (rb -> {
         if (rb.getName ().equals ("ground"))
            return;
         output
            .append (rb.getName () + "\t")
            .append (String.format ("%.3f", rb.getMass ())).append ("\tkg\n");
         MeshComponentList<RigidMeshComp> meshes = rb.getMeshComps ();
         // skip first component, since it does not contain any useful info
         for (int i = 1; i < meshes.size (); i++) {
            RigidMeshComp mesh = meshes.get (i);
            output.append(mesh.getName ()).append ("\n");
            ArrayList<Vertex3d> vertices =
               mesh.getSurfaceMesh ().getVertices ();
            vertices.forEach (vt -> {
               output
                  .append (vt.getIndex ()).append ("\t")
                  .append (vt.getPosition ().toString ("%.3f")).append ("\n");
            });
            ArrayList<Face> faces = mesh.getSurfaceMesh ().getFaces ();
            faces.forEach (f -> {
               output
                  .append (f.idx).append ("\t")
                  .append (f.getVertex (0).getIndex ()).append ("\t")
                  .append (f.getVertex (1).getIndex ()).append ("\t")
                  .append (f.getVertex (2).getIndex ()).append ("\t")
                  .append ("\n");
            });
         }
         output.append ("\n");
      });
   }

   /**
    * Adds muscle information to the output string.
    * 
    * @param output
    * {@link StringBuilder} string
    * @param muscles
    * list of {@link MultiPointMuscle} objects
    */
   private void writeMuscleInfo (
      StringBuilder output, List<MuscleComponent> muscles) {
      if (muscles == null)
         return;
      output
         .append ("MUSCLES\n").append ("Number of muscles: ")
         .append (muscles.size ()).append ("\n\n");
      muscles.forEach (msc -> {
         AxialMaterial mat = msc.getMaterial ();
         output
            .append ("Name: ").append (msc.getName ()).append ("\n")
            .append ("Material: ").append (mat.getClass ().getSimpleName ())
            .append ("\n");
         if (msc.getMaterial () instanceof EquilibriumAxialMuscle) {
            EquilibriumAxialMuscle emat =
               (EquilibriumAxialMuscle)msc.getMaterial ();
            output
               .append ("Max isometric force: ").append (emat.getMaxIsoForce ())
               .append ("\n").append ("Rest length: ")
               .append (String.format ("%.3f", msc.getRestLength ()))
               .append ("\n").append ("Opt fiber length: ")
               .append (String.format ("%.3f", emat.getOptFibreLength ()))
               .append ("\n").append ("Tendon slack length: ")
               .append (String.format ("%.3f", emat.getTendonSlackLength ()))
               .append ("\n").append ("Number of points: ")
               .append (msc.numPoints ()).append ("\n");
         }
         if (msc instanceof MultiPointMuscle) {
            MultiPointMuscle multi = (MultiPointMuscle)msc;
            for (int i = 0; i < msc.numPoints (); i++) {
               Point point = multi.getPoint (i);
               String frameMarkerString =
                  String
                     .format (
                        "%-6s%-2s%-23s%-19s", "Point", i,
                        point.getClass ().getSimpleName (),
                        point.getPosition ().toString ("%.3f"));
               output.append (frameMarkerString);
               if (point instanceof JointConditionalMarker) {
                  DoubleInterval range =
                     ((JointConditionalMarker)point).getRange ();
                  output.append (range.toString ()).append ("\n");
               }
               else if (point instanceof JointBasedMovingMarker) {
                  Diff1Function1x1[] fcts =
                     ((JointBasedMovingMarker)point).getCoordinateFunctions ();
                  for (int j = 0; j < fcts.length; j++) {
                     String coords[] = { "x", "y", "z" };
                     ScaledDiff1Function1x1 fct =
                        (ScaledDiff1Function1x1)fcts[j];
                     if (fct.getFunction () instanceof CubicHermiteSpline1d) {
                        output
                           .append ("\nCubic hermite spline knots X Y\n")
                           .append (coords[j]).append (" location\n");
                        CubicHermiteSpline1d hermite =
                           (CubicHermiteSpline1d)fct.getFunction ();
                        for (int k = 0; k < hermite.numKnots (); k++) {
                           double x = hermite.getKnot (k).getX ();
                           double y = hermite.getKnot (k).getY ();
                           String knots =
                              String.format ("%.3f%-1s%.3f%n", x, " ", y);
                           output.append (knots);
                        }
                     }
                  }
               }
               else {
                  output.append ("\n");
               }
            }
            output
               .append ("Number of segments: ").append (multi.numSegments ())
               .append ("\n").append ("Number of muscle wrappings: ")
               .append (multi.numWrappables ()).append ("\n");
            for (int i = 0; i < multi.numWrappables (); i++) {
               output
                  .append ("Wrapping: ").append (i).append ("\t")
                  .append (multi.getWrappableRange (i)).append ("\n");
            }
         }
         output.append ("\n");
      });
   }

   /**
    * Adds physics information to the output string.
    * 
    * @param output
    * {@link StringBuilder} string
    * @param mech
    * current {@link MechModel}
    */
   private void writePhysicsInfo (StringBuilder output, MechModel mech) {
      output
         .append ("\nPHYSICS\n").append ("Frame Damping: ")
         .append (mech.getFrameDamping ()).append ("\n")
         .append ("Rotary Damping: ").append (mech.getRotaryDamping ())
         .append ("\n").append ("Inertial Damping: ")
         .append (mech.getInertialDamping ()).append ("\n")
         .append ("Point Damping: ").append (mech.getPointDamping ())
         .append ("\n").append ("Gravity: ").append (mech.getGravity ())
         .append ("\n");
   }

   /**
    * Adds all probes information to the output string.
    * 
    * @param output
    * {@link StringBuilder} string
    * @param controller
    * current {@link MotionTargetController} object
    * @param motion
    * {@link MarkerMotionData} object containing all exp trajectories
    * @param map
    * {@link Map} linking the experimental marker names and weights to model
    * markers
    * @param forces
    * {@link ForceData} object containing all exp forces
    * @param marker
    * list of {@link FrameMarker} objects
    */
   private void writeProbesInfo (
      StringBuilder output, MotionTargetController controller,
      MarkerMotionData motion, MarkerMapping map, ForceData forces,
      RenderableComponentList<FrameMarker> marker) {
      output
         .append ("\n%%PROBES%%\n")
         .append (
            "%%------------------------------------------------------------%%\n")
         .append ("\nTRACKING CONTROLLER INFORMATION\n")
         .append ("Performed IK at startup: ").append (myFilterFDWithIK).append ("\n")
         .append ("Use KKT Factorization: ")
         .append (controller.getUseKKTFactorization ()).append ("\n")
         .append ("Incremental computation: ")
         .append (controller.getComputeIncrementally ()).append ("\n")
         .append ("Use timestep scaling: ")
         .append (controller.getUseTimestepScaling ()).append ("\n")
         .append ("Normalize cost terms: ")
         .append (controller.getNormalizeCostTerms ()).append ("\n");
      // Motion Target Term
      MotionTargetTerm mTerm = controller.getMotionTargetTerm ();
      output
         .append ("\nMOTION TARGET TERM INFORMATION\n").append ("Weight: ")
         .append (mTerm.getWeight ()).append ("\n")
         .append ("Use legacy control: ").append (mTerm.getLegacyControl ())
         .append ("\n").append ("Use PD control: ")
         .append (mTerm.getUsePDControl ()).append ("\n")
         .append ("Chase Time: ").append (mTerm.getChaseTime ()).append ("\n")
         .append ("KD: ").append (mTerm.getKd ()).append ("\n").append ("KP: ")
         .append (mTerm.getKp ()).append ("\n");
      // Damping Term
      DampingTerm dTerm = controller.getExcitationDampingTerm ();
      if (dTerm != null) {
         output
            .append ("\nDAMPING TERM INFORMATION\n").append ("Enabled: ")
            .append (dTerm.isEnabled ()).append ("\n").append ("Weight: ")
            .append (dTerm.getWeight ()).append ("\n");
      }
      // L2Regularization Term
      L2RegularizationTerm lTerm = controller.getL2RegularizationTerm ();
      if (lTerm != null) {
         output
            .append ("\nL2REGULARIZATION TERM INFORMATION\n")
            .append ("Enabled: ").append (lTerm.isEnabled ()).append ("\n")
            .append ("Weight: ").append (lTerm.getWeight ()).append ("\n");
      }
      // Fallback solver to run the cost term solver when constrained solves
      // aren't feasible
      FeasibleQPSolver solver = controller.getFeasibleSolver ();
      if (solver != null) {
         output
            .append ("\nFEASIBLE SOLVER INFORMATION\n").append ("Enabled: ")
            .append (solver.isEnabled ()).append ("\n")
            .append ("Bounds threshold: ")
            .append (solver.getBoundsThreshold ()).append ("\n")
            .append ("Fallback Q per unit weight: ")
            .append (solver.getFallbackQMuscle ()).append ("\n")
            .append ("Fallback L2 weight: ")
            .append (solver.getFallbackL2Weight ()).append ("\n")
            .append ("Equality tolerance: ")
            .append (solver.getEqualityTolerance ()).append ("\n");
      }
      // Force excitation Term
      ForceExcitationTargetTerm fTerm =
         controller.getForceExcitationTargetTerm ();
      if (fTerm != null) {
         output
            .append ("\nFORCE EXCITATION TARGET TERM INFORMATION\n")
            .append ("Enabled: ").append (fTerm.isEnabled ()).append ("\n")
            .append ("Weight: ").append (fTerm.getWeight ()).append ("\n");
         output.append (String.format ("%-26s%-6s%-6s%n", "Name", "Value", "Weight"));
         for (int i = 0; i < fTerm.numTargets (); i++) {
            double value = fTerm.getTargetValue (i);
            String name = fTerm.getTargetComponent (i).getName ();
            double weight = fTerm.getTargetWeight(i);
            output.append (String.format ("%-26s%-6s%-6s%n", name, value, weight));
         }
      }
      // Used Targets
      String useFrameTargets =
         (mTerm.getTargetFrames ().size () != 0) ? "true" : "false";
      String usePointTargets =
         (mTerm.getTargetPoints ().size () != 0) ? "true" : "false";
      output
         .append ("\nTARGET INFORMATION\n").append ("Use Frame Targets: ")
         .append (useFrameTargets).append ("\n").append ("Use Point Targets: ")
         .append (usePointTargets).append ("\n");
      output
         .append ("Model markers: ").append (marker.size ()).append ("\t")
         .append ("Assigned experimental markers: ")
         .append (motion.numMarkers ()).append ("\n");
      // Marker information
      String markerHeader =
         String
            .format (
               "%-16s%-16s%-16s%-6s", "Model marker", "Exp. marker",
               "Attachment", "Weight");
      output.append (markerHeader).append ("\n");
      // Add marker pairs
      marker.forEach (mkr -> {
         try {
            String label = map.getExpLabelFromModel (mkr.getName ());
            String appendix =
               (label == null)
                  ? String
                     .format (
                        "%-16s%-46s%n", mkr.getName (),
                        "No corresponding experimental marker detected.")
                  : String
                     .format (
                        "%-16s%-16s%-16s%.3f%n", mkr.getName (), label,
                        mkr.getFrame ().getName (),
                        map.getMarkerWeight (label));
            output.append (appendix);
         }
         catch (Exception e) {
            e.printStackTrace ();
         }
      });
      output.append ("\nPROBE INFORMATION\n");
      // Calculate framerates for marker and force data
      double framerate =
         (motion.numFrames () - 1)
         / motion.getFrameTime (motion.numFrames () - 1);
      String trcFormat =
         "TRC File: %d, start time: %.3f, stop time: %.3f, frame rate: %.2f%n";
      output
         .append (
            String
               .format (
                  trcFormat, motion.numFrames (), motion.getFrameTime (0),
                  motion.getFrameTime (motion.numFrames () - 1), framerate));
      framerate =
         (forces.numFrames () - 1)
         / forces.getFrameTime (forces.numFrames () - 1);
      String forceFormat =
         "MOT File: %d, start time: %.3f, stop time: %.3f, frame rate: %.2f%n";
      output
         .append (
            String
               .format (
                  forceFormat, forces.numFrames (), forces.getFrameTime (0),
                  forces.getFrameTime (forces.numFrames () - 1), framerate));
      // List all input probes
      ComponentList<Probe> iProbes = getInputProbes ();
      output
         .append ("Generated ").append (iProbes.size ())
         .append (" input probes from trc and mot file(s)").append ("\n");
      String targetHeader =
         String.format ("%-32s%-22s%-8s", "Name", "Type", "Enabled");
      output.append (targetHeader).append ("\n");
      for (Probe probe : iProbes) {
         String appendix =
            String
               .format (
                  "%-32s%-22s%-6s%n", probe.getName (),
                  probe.getClass ().getSimpleName (), probe.isActive ());
         output.append (appendix);
      }
      output.append ("\nEXCITER INFORMATION\n");
      String exHeader =
         String
            .format (
               "%-26s%-22s%-8s%-12s%-6s%-6s", "Name", "Type", "Weight",
               "Force scale", "a_min", "a_max");
      output.append (exHeader).append ("\n");
      ListView<ExcitationComponent> exciters = controller.getExciters ();
      for (ExcitationComponent ex : exciters) {
         double weight = controller.getExcitationWeight (ex);
         DoubleInterval bound = controller.getExcitationBounds (ex);
         double scale = 0;
         if (ex instanceof JointActuator) {
            JointActuator jAct = (JointActuator)ex;
            scale = jAct.getForceScale ();
         }
         String exFormat = "%-26s%-22s%-8s%-12s%-6s%-6s%n";
         output
            .append (
               String
                  .format (
                     exFormat, ex.getName (), ex.getClass ().getSimpleName (),
                     weight, scale, bound.getLowerBound (),
                     bound.getUpperBound ()));
      }
   }

   /**
    * Adds solver information to the output string.
    * 
    * @param output
    * {@link StringBuilder} string
    * @param solver
    * {@link MechSystemSolver} of the current {@link MechModel}
    */
   private void writeSolverInfo (
      StringBuilder output, MechSystemSolver solver) {
      output
         .append ("\n%%GENERAL SIMULATION PARAMETERS%%\n")
         .append (
            "%%------------------------------------------------------------%%\n")
         .append ("\nSOLVER\n").append ("Integrator: ")
         .append (solver.getIntegrator ().toString ()).append ("\n")
         .append ("Stabilization: ")
         .append (solver.getStabilization ().toString ()).append ("\n")
         .append ("Max Iterations: ").append (solver.getMaxIterations ())
         .append ("\n").append ("Max Tolerance: ")
         .append (solver.getTolerance ()).append ("\n")
         .append ("Matrix Solver: ")
         .append (solver.getMatrixSolver ().toString ()).append ("\n")
         .append ("Hybrid Solving enabled: ").append (solver.getHybridSolve ())
         .append ("\n");
   }
}