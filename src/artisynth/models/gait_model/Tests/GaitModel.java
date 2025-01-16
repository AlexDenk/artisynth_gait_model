package artisynth.models.gait_model.Tests;

import java.awt.Color;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Map;

import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.JLabel;

import artisynth.core.driver.Main;
import artisynth.core.femmodels.*;
import artisynth.core.femmodels.FemModel.Ranging;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.gui.ControlPanel;
import artisynth.core.inverse.*;
import artisynth.core.inverse.FrameExciter.WrenchDof;
import artisynth.core.inverse.InverseManager.ProbeID;
import artisynth.core.materials.*;
import artisynth.core.mechmodels.*;
import artisynth.core.mechmodels.MechSystemSolver.Integrator;
import artisynth.core.mechmodels.MechSystemSolver.PosStabilization;
import artisynth.core.modelbase.*;
import artisynth.core.opensim.OpenSimParser;
import artisynth.core.opensim.components.ForceSpringBase;
import artisynth.core.opensim.customjoint.OpenSimCustomJoint;
import artisynth.core.probes.*;
import artisynth.core.renderables.ColorBar;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.RootModel;
import artisynth.models.gait_model.*;

import maspack.geometry.*;
import maspack.interpolation.Interpolation;
import maspack.matrix.*;
import maspack.render.*;
import maspack.render.Renderer.FaceStyle;
import maspack.render.Renderer.LineStyle;
import maspack.render.Renderer.Shading;
import maspack.render.Viewer.RotationMode;
import maspack.spatialmotion.Wrench;
import maspack.util.*;

/**
 * @author Alexander Denk Copyright (c) 2023-2024 <br>
 * (UDE) University of Duisburg-Essen <br>
 * Chair of Mechanics and Robotics <br>
 * alexander.denk@uni-due.de
 */

public class GaitModel extends RootModel {
   // ----------------------------Instance Fields-------------------------------
   // All rigid bodies in the model
   RenderableComponentList<RigidBody> myBodies = null;
   // Calculated generalized joint angles
   CoordinateData myCoords;
   // Experimental force data
   ForceData myForces;
   // A list of all joints in the model
   RenderableComponentList<JointBase> myJoints;
   // All markers in the model
   RenderableComponentList<FrameMarker> myMarkers = null;
   // Current mechmodel
   MechModel myMech = new MechModel ();
   // All finite element meshes in the model
   List<FemModel3d> myMeshes = new ArrayList<FemModel3d> ();
   // Experimental and model marker names and weights
   MarkerMapping myMap;
   // Experimental marker trajectories
   MarkerMotionData myMotion;
   // A list of each muscle in the model
   List<MuscleComponent> myMuscles = new ArrayList<MuscleComponent> ();
   // Name of the current working directory
   String myName = null;
   // Scale of the model.
   double myScale;
   // Used to track whether inverse kinematics is used
   boolean myUseIK = true;
   // Used to track whether ground reaction forces are used
   boolean myUseGRF = true;
   // Arrays of grouped muscles, sorted by location and function
   String[] gluts =
      new String[] { "glut_med1_r",
                     "glut_med2_r",
                     "glut_med3_r",
                     "glut_min1_r",
                     "glut_min2_r",
                     "glut_min3_r",
                     "glut_max1_r",
                     "glut_max2_r",
                     "glut_max3_r",
                     "glut_med1_l",
                     "glut_med2_l",
                     "glut_med3_l",
                     "glut_min1_l",
                     "glut_min2_l",
                     "glut_min3_l",
                     "glut_max1_l",
                     "glut_max2_l",
                     "glut_max3_l"};
   String[] pelvisFemur =
      new String[] { "add_long_r",
                     "add_brev_r",
                     "add_mag1_r",
                     "add_mag2_r",
                     "add_mag3_r",
                     "pect_r",
                     "iliacus_r",
                     "psoas_r",
                     "quad_fem_r",
                     "gem_r",
                     "peri_r",
                     "add_long_l",
                     "add_brev_l",
                     "add_mag1_l",
                     "add_mag2_l",
                     "add_mag3_l",
                     "pect_l",
                     "iliacus_l",
                     "psoas_l",
                     "quad_fem_l",
                     "gem_l",
                     "peri_l"};
   String[] pelvisTibia =
      new String[] { "semimem_r",
                     "semiten_r",
                     "bifemlh_r",
                     "sar_r",
                     "tfl_r",
                     "grac_r",
                     "rect_fem_r",
                     "semimem_l",
                     "semiten_l",
                     "bifemlh_l",
                     "sar_l",
                     "tfl_l",
                     "grac_l",
                     "rect_fem_l"};
   String[] femurTibia =
      new String[] { "bifemsh_r",
                     "vas_med_r",
                     "vas_int_r",
                     "vas_lat_r",
                     "bifemsh_l",
                     "vas_med_l",
                     "vas_int_l",
                     "vas_lat_l"};
   String[] problemMuscles =
      new String[] { "rect_fem_l",
                     "rect_fem_r",
                     "vas_med_r",
                     "vas_lat_r",
                     "vas_med_l",
                     "vas_lat_l",
                     "ext_dig_r",
                     "ext_dig_l"};
   String[] femurCalcn =
      new String[] { "med_gas_r",
                     "lat_gas_r",
                     "med_gas_l",
                     "lat_gas_l"};
   String[] tibiaCalcn =
      new String[] { "soleus_r",
                     "tib_post_r",
                     "tib_ant_r",
                     "per_brev_r",
                     "per_long_r",
                     "per_tert_r",
                     "soleus_l",
                     "tib_post_l",
                     "tib_ant_l",
                     "per_brev_l",
                     "per_long_l",
                     "per_tert_l"};
   String[] tibiaToes =
      new String[] { "flex_dig_r",
                     "flex_hal_r",
                     "ext_dig_r",
                     "ext_hal_r",
                     "flex_dig_l",
                     "flex_hal_l",
                     "ext_dig_l",
                     "ext_hal_l"};
   String[] pelvisTorso =
      new String[] { "extobl_r",
                     "intobl_r", 
                     "ercspn_r",
                     "extobl_l",
                     "intobl_l",
                     "ercspn_l"};
   // ----------------------------Nested classes--------------------------------
   public class MomentArmFunction
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

      public MomentArmFunction (Frame frame, String side, double s) {
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
         writeToFile (wrench);
         // calculate endpoint of grf vector for rendering
         grfEndPos.scaledAdd (0.001 * myScale, grf, copPos);
      }

      private void writeToFile (Wrench wrench) {
         StringBuilder message = new StringBuilder ();
         message
            .append ("\nCOMPUTED WRENCH FOR: ")
            .append (myFrame.getName ().toUpperCase ()).append ("\n");
         message.append (wrench.toString ("%.3f")).append ("\n");
         writer.print (message);
         writer.flush ();
         message.delete (0, message.length ());
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

   public class JointMomentFunction implements DataFunction, Clonable {
      JointBase myJoint;
      String myJointConstraint;
      double myMass;

      public JointMomentFunction (JointBase joint, String constraint) {
         this.myJoint = joint;
         this.myJointConstraint = constraint;
         this.myMass = myMech.getActiveMass ();
      }

      public Object clone () throws CloneNotSupportedException {
         return super.clone ();
      }

      public void eval (VectorNd vec, double t, double trel) {
         Vector3d uniBufInA = myJoint.getUnilateralForceInA ().m;
         Vector3d biBufInA = myJoint.getBilateralForceInA ().m;
         VectorNd mom = new VectorNd (1);
         switch (myJoint.getName ()) {
            case "back":
               if (myJointConstraint.contains ("extension"))
                  mom.set (0, biBufInA.z + uniBufInA.z);
               if (myJointConstraint.contains ("bending"))
                  mom.set (0, biBufInA.x + uniBufInA.x);
               if (myJointConstraint.contains ("rotation"))
                  mom.set (0, biBufInA.y + uniBufInA.y);
               break;
            case "hip_r":
            case "hip_l":
               if (myJointConstraint.contains ("flexion"))
                  mom.set (0, biBufInA.z + uniBufInA.z);
               if (myJointConstraint.contains ("adduction"))
                  mom.set (0, biBufInA.x + uniBufInA.x);
               if (myJointConstraint.contains ("rotation"))
                  mom.set (0, biBufInA.y + uniBufInA.y);
               break;
            case "knee_r":
            case "knee_l":
            case "ankle_r":
            case "ankle_l":
            case "mtp_r":
            case "mtp_l":
               mom.set (0, biBufInA.z + uniBufInA.z);
               break;
            case "subtalar_r":
            case "subtalar_l":
               mom.set (0, biBufInA.x + uniBufInA.x);
               break;
         }
         vec.add (mom);
         // vec.scaledAdd (1 / myMass, mom);
      }
   }

   // -----------------------------Constructors---------------------------------
   public GaitModel () {
   }

   public GaitModel (String name) throws IOException {
      super (name);
   }

   public GaitModel (String name, boolean ik, boolean grf) throws IOException {
      super (name);
      this.myUseIK = ik;
      this.myUseGRF = grf;
   }

   // --------------------------Static Methods----------------------------------
   public static void main (String[] args) throws IOException {
   }

   // -------------------------Instance Methods---------------------------------
   @Override
   public void build (String[] args) throws IOException {
      super.build (args);
      // Get model name specifier from user
      JFileChooser fc = new JFileChooser ();
      myName = getNameFromFileDiaglog (fc);
      myMech.setName (myName);
      addModel (myMech);
      setSimulationProperties ();
      initializeOsim (myName, myScale);
      initializeIOProbes (myName, myScale);
      CollisionManager collMan = myMech.getCollisionManager ();
      setContactProps (collMan, myName);
      setRenderProps (collMan, myScale);
      writeInputToFile (myName);
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
      markers.forEach (m -> {
         RenderProps.setPointColor (m, Color.PINK);
      });
      // Access source and target points of the motion target controller
      ComponentListView<Controller> controllers = getControllers ();
      if (controllers.size () == 0)
         return;
      TrackingController controller =
         (TrackingController)controllers.get ("Motion controller");
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
      getMainViewer ().setOrthographicView (true);
      getMainViewer ().setRotationMode (RotationMode.CONTINUOUS);
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
      String inputName = myName + "/Output/" + myName + "_input_file.txt";
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
               "%% Author: Alexander Denk, Copyright (c) 2024                 %%\n")
            .append (
               "%% (UDE) University of Duisburg-Essen                         %%\n")
            .append (
               "%% Chair of Mechanics and Robotics                            %%\n")
            .append (
               "%% alexander.denk@uni-due.de                                  %%\n")
            .append (
               "%%------------------------------------------------------------%%\n");
         MechSystemSolver solver = myMech.getSolver ();
         writeSolverInfo (output, solver);
         writePhysicsInfo (output, myMech);
         String modelPath = myName + "/" + myName + "_scaled.osim";
         writeModelInfo (output, modelPath, myBodies);
         writeJointInfo (output, myJoints);
         writeMuscleInfo (output, myMuscles);
         writeFEMInfo (output, myMeshes);
         TrackingController controller =
            (TrackingController)getControllers ().get ("Motion controller");
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
    * Defines a tracking controller that calculates muscle activations based on
    * trajectories and sets its controller properties.
    * 
    * @param motion
    * {@link MarkerMotionData}
    * @param map
    * {@link MarkerMapping}
    * @param name
    * Name specifier of the current working directory
    */
   private TrackingController addControllerAndProps () {
      TrackingController controller =
         new TrackingController (myMech, "Motion controller");
      controller.setUseKKTFactorization (false);
      controller.setComputeIncrementally (false);
      controller.setExcitationDamping ();
      controller.setL2Regularization (0.001);
      // Adjust MotionTargetTerm properties
      MotionTargetTerm motionTerm = controller.getMotionTargetTerm ();
      motionTerm.setUsePDControl (true);
      motionTerm.setChaseTime (getMaxStepSize ());
      motionTerm.setKd (0.1 / getMaxStepSize ());
      motionTerm.setKp (1 / (getMaxStepSize () * motionTerm.getChaseTime ()));
      controller.createPanel (this);
      addController (controller);
      return controller;
   }

   /**
    * Adds the data stored in {@code myCoords} as a {@link NumericInputProbe}
    * per joint, if available.
    * 
    * @param coords
    * {@link CoordinateData}
    * @param start
    * start time
    * @param stop
    * stop time
    */
   private void addCoordsInputProbes (
      CoordinateData coords, double start, double stop) {
      if (coords != null) {
         myJoints.forEach (jt -> {
            for (int j = 0; j < jt.numCoordinates (); j++) {
               createCoordsInputProbe (
                  jt, coords, jt.getCoordinateName (j), start, stop);
            }
         });
      }
   }
   
   /**
    * Adds the position error as numeric output probe
    * 
    * @param controller
    * {@link TrackingController}
    * @param start
    * start time
    * @param stop
    * stop time
    * @param step
    * step size
    */
   private void addErrorOutputProbes (
      TrackingController controller, double start, double stop, double step) {
      MotionTargetTerm mTerm = controller.getMotionTargetTerm ();
      createProbeAndPanel (mTerm, null, "positionError", start, stop, step);
   }

   /**
    * Adds all model muscles and three frame exciters each for pelvis and both
    * calcanei to the controller.
    * 
    * @param controller
    * {@link TrackingController}
    */
   private void addExcitersToController (TrackingController controller) {
      // Muscles
      //controller.addExciters (myMuscles);
      // Frame Exciters
      HashSet<String> bodyNames = new HashSet<> ();
      bodyNames.add ("torso");
      bodyNames.add ("pelvis");
      bodyNames.add ("femur_r");
      bodyNames.add ("femur_l");
      bodyNames.add ("tibia_r");
      bodyNames.add ("tibia_l");
      bodyNames.add ("talus_r");
      bodyNames.add ("talus_l");
      bodyNames.add ("calcn_r");
      bodyNames.add ("calcn_l");
      bodyNames.add ("toes_r");
      bodyNames.add ("toes_l");
      myBodies.forEach (body -> {
         if (!bodyNames.contains (body.getName ()))
            return;
         if (body.getMass () == 0)
            return;
         double maxForce;
         double maxMoment;
         double[] scale = new double[] { 1, 1, 1, 1, 1, 1 };
         double[] weights = new double[] { 1, 1, 1, 1, 1, 1 };
         switch (body.getName ()) {
            case "calcn_l":
            case "calcn_r":
               //maxForce =
               //   2 * myMech.getGravity ().norm () * myMech.getActiveMass ();
               maxForce = 300;
               maxMoment = maxForce;
               //scale[0] = 1;
               //scale[1] = 1;
               //scale[2] = 1;
               //scale[3] = 0.3;
               //scale[4] = 0.3;
               //scale[5] = 1;
               //weights[0] = 1e0;
               //weights[1] = 1e0;
               //weights[2] = 1e0;
               //weights[3] = 1e0;
               //weights[4] = 1e0;
               //weights[5] = 1e0;
               break;
            default:
               maxForce = 40 * body.getMass ();
               maxMoment = maxForce;
               //scale[0] = 1;
               //scale[1] = 1;
               //scale[2] = 1;
               //scale[3] = 1;
               //scale[4] = 1;
               //scale[5] = 1;
               //weights[0] = 1e0;
               //weights[1] = 1e0;
               //weights[2] = 1e0;
               //weights[3] = 1e0;
               //weights[4] = 1e0;
               //weights[5] = 1e0;
               break;
         }
         createAndAddFrameExciters (
            controller, myMech, body, maxForce, maxMoment, scale, weights);
      });
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
    * @param stop
    * stop time
    * @param step
    * step size
    */
   private void addJointAngleOutputProbes (
      double start, double stop, double step) {
      ControlPanel jointPanel = new ControlPanel ("Joint Coordinates");
      myJoints.forEach (jt -> {
         for (int i = 0; i < jt.numCoordinates (); i++) {
            createProbeAndPanel (
               jt, jointPanel, jt.getCoordinateName (i), start, stop, step);
            jointPanel
               .addWidget (
                  jt.getName () + " locked", jt,
                  jt.getCoordinateName (i) + "_locked");
         }
      });
      addControlPanel (jointPanel);
   }

   /**
    * Adds output probes for joint forces and moments
    * 
    * @param start
    * start time
    * @param stop
    * stop time
    * @param step
    * step size
    */
   private void addJointLoadOutputProbes (
      double start, double stop, double step) {
      myJoints.forEach (joint -> {
         for (int i = 0; i < joint.numCoordinates (); i++) {
            String probeName =
               joint.getName () + " " + joint.getCoordinateName (i) + "_moment";
            String filepath =
               PathFinder
                  .getSourceRelativePath (
                     this, "/" + myName + "/Output/" + probeName + ".txt");
            NumericMonitorProbe momProbe =
               new NumericMonitorProbe (1, filepath, start, stop, step);
            momProbe.setModel (myMech);
            momProbe.setName (probeName);
            momProbe.setInterpolationOrder (Interpolation.Order.Cubic);
            JointMomentFunction momentSum =
               new JointMomentFunction (joint, joint.getCoordinateName (i));
            momProbe.setDataFunction (momentSum);
            addOutputProbe (momProbe);
         }
      });
   }

   /**
    * Adds output probes and panel widgets for muscle excitations and frame
    * exciter max force/moment.
    * 
    * @param controller
    * {@link TrackingController}
    * @param start
    * start time
    * @param stop
    * stop time
    * @param step
    * step size
    */
   private void addMuscleOutputProbes (
      TrackingController controller, double start, double stop, double step) {
      ControlPanel musclePanel = new ControlPanel ("Muscle Properties");
      controller.getExciters ().forEach (e -> {
         if (e instanceof FrameExciter) {
            // createProbeAndPanel (
            // e, musclePanel, "excitation", start, stop, step);
            musclePanel
               .addWidget (e.getName () + "excitation", e, "excitation");
            // for debugging
            musclePanel.addWidget (e.getName () + " maxForce", e, "maxForce");
         }
         else {
            createProbeAndPanel (e, null, "forceNorm", start, stop, step);
            musclePanel
               .addWidget (e.getName () + " excitation", e, "excitation");
         }
      });
      addControlPanel (musclePanel);
   }

   /**
    * Creates the default {@link NumericOutputProbe} of the
    * {@link TrackingController} and fills a control panel with all
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
      MarkerMotionData motion, TrackingController controller, double start, double stop) {
      if (motion == null)
         return;
      double step = getMaxStepSize ();
      if (controller != null) {
         setDefaultOutputProbes (controller, start, stop);
         addErrorOutputProbes(controller, start, stop, step);
         addMuscleOutputProbes (controller, start, stop, step);
      }
      addJointAngleOutputProbes (start, stop, step);
      addJointLoadOutputProbes(start, stop, step);
   }

   /**
    * Defines the point targets of the {@link MotionTargetTerm} of the
    * {@link TrackingController} and creates a {@link NumericInputProbe} for
    * each target.
    * 
    * @param controller
    * TrackingController
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
    */
   private void addPointTargetsAndProbes (
      TrackingController controller, MarkerMapping map, MarkerMotionData motion,
      double start, double stop)
      throws IOException {
      if (motion == null || map == null)
         return;
      // Add markers to be targets and set their positions
      VectorNd weights = new VectorNd ();
      ArrayList<FrameMarker> targets = new ArrayList<FrameMarker> ();
      for (int i = 0; i < myMarkers.size (); i++) {
         FrameMarker mkr = myMarkers.get (i);
         String markerName = mkr.getName ();
         String expName = map.getExpLabelFromModel (markerName);
         if (expName != null) {
            Double weight = map.getMarkerWeight (markerName);
            weights.append (weight);
            targets.add (mkr);
            TargetPoint target = controller.addPointTarget (mkr, weight);
            // Adjust initial position of each target
            Point3d position = (Point3d)motion.getMarkerPosition (0, expName);
            target.setPosition (position);
         }
      }
      // Add the default MarkerMotionData prior to IK solving
      NumericInputProbe targetProbe =
         collectMarkerMotionData (
            controller, targets, motion, map, start, stop);
      addInputProbe (targetProbe);
      if (myUseIK) {
         // Generate achievable target positions by IK
         PositionInputProbe posProbe =
            createIKPositions (targets, weights, targetProbe);
         // Overwrite default marker positions with achievable ones
         targetProbe.setValues (posProbe, true);
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
    * TrackingController
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
      TrackingController controller, ArrayList<FrameMarker> targets,
      MarkerMotionData motion, MarkerMapping map, double start, double stop) {
      NumericInputProbe targetProbe =
         InverseManager.createInputProbe (
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
    * Creates a complete set of FrameExciters for a given frame and adds them to
    * a MechModel and a tracking controller.
    *
    * @param ctrl
    * tracking controller to add the exciters to
    * @param mech
    * MechModel to add the exciters to
    * @param frame
    * frame for which the exciters should be created
    * @param maxForce
    * maximum translational force along any axis
    * @param maxMoment
    * maximum moment about any axis
    * @param excScales
    * factors to scale max force and moment to individual wrench dofs
    * @param regWeights
    * exciter weights for the inverse controller
    * @author John Lloyd
    */
   private FrameExciter[] createAndAddFrameExciters (
      TrackingController ctrl, MechModel mech, Frame frame, double maxForce,
      double maxMoment, double[] excScales, double[] regWeights) {
      FrameExciter[] exs = new FrameExciter[6];
      exs[0] =
         new FrameExciter (null, frame, WrenchDof.FX, excScales[0] * maxForce);
      exs[1] =
         new FrameExciter (null, frame, WrenchDof.FY, excScales[1] * maxForce);
      exs[2] =
         new FrameExciter (null, frame, WrenchDof.FZ, excScales[2] * maxForce);
      exs[3] =
         new FrameExciter (null, frame, WrenchDof.MX, excScales[3] * maxMoment);
      exs[4] =
         new FrameExciter (null, frame, WrenchDof.MY, excScales[4] * maxMoment);
      exs[5] =
         new FrameExciter (null, frame, WrenchDof.MZ, excScales[5] * maxMoment);
      // if the frame has a name, use this to create names for the exciters
      if (frame.getName () != null) {
         WrenchDof[] wcs = WrenchDof.values ();
         for (int i = 0; i < exs.length; i++) {
            exs[i]
               .setName (
                  frame.getName () + "_" + wcs[i].toString ().toLowerCase ());
         }
      }
      if (mech != null) {
         for (int i = 0; i < exs.length; i++) {
            mech.addForceEffector (exs[i]);
            ctrl.addExciter (regWeights[i], exs[i]);
         }
      }
      return exs;
   }

   /**
    * Adds a collision response {@code collResp} and behavior {@code collBehav}
    * object for the compliant contact between {@code bodyA} and {@code bodyB},
    * based on the given parameters {@code comp} and {@code damp}.
    * 
    * @param bodyA
    * @param bodyB
    * @param comp
    * Compliance coefficient
    * @param damp
    * Damping coefficient
    */
   private void createCollision (
      RigidBody bodyA, RigidBody bodyB, double comp, double damp) {
      CollisionBehavior behavior;
      behavior = myMech.setCollisionBehavior (bodyA, bodyB, true);
      behavior.setCompliance (comp);
      behavior.setDamping (damp);
      myMech.setCollisionResponse (bodyA, bodyB);
   }

   /**
    * Creates a {@link NumericInputProbe} for each {@link JointBase}
    * {@code joint} coordinate specified by {@code prop} and fills it with the
    * angles in {@code coords}.
    * 
    * @param joint
    * member of {@code myJoints}
    * @param coords
    * {@link CoordinateData}
    * @param prop
    * joint coordinate
    * @param start
    * probe start time
    * @param stop
    * probe stop time
    */
   private void createCoordsInputProbe (
      JointBase joint, CoordinateData coords, String prop, double start,
      double stop) {
      NumericInputProbe angle =
         new NumericInputProbe (joint, prop, start, stop);
      angle.setModel (myMech);
      angle.setName (prop);
      for (int i = 0; i < coords.numFrames (); i++) {
         double time = coords.getFrameTime (i);
         double[] coord = new double[1];
         coord[0] = coords.getData (i, prop);
         angle.addData (time, coord);
      }
      angle.setActive (true);
      addInputProbe (angle);
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
      ForceData forces, Frame frame, String side, double start, double stop, double scale) {
      NumericControlProbe grf = new NumericControlProbe ();
      grf.setModel (myMech);
      grf.setName (frame.getName () + " ground reaction forces");
      grf.setStartStopTimes (start, stop);
      grf.setInterpolationOrder (Interpolation.Order.Cubic);
      MomentArmFunction momentArm = new MomentArmFunction (frame, side, scale);
      Main.getMain ().getViewer ().addRenderable (momentArm);
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
    * Generates achievable positions for the PointTargets in {@code targets} by
    * solving an inverse kinematics problem. The calculated new PointTarget
    * positions are then used as actual target positions by the
    * TrackingController.
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
      PositionInputProbe posProbe =
         solver.createMarkerPositionProbe (posProbeName, targetProbe, -1);
      VectorNd targs0 = targetProbe.getNumericList ().getFirst ().v;
      int niters = solver.solve (targs0);
      System.out.println ("in " + niters + " iterattions");
      return posProbe;
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
    * @param start
    * Start time of the probe
    * @param stop
    * Stop time of the probe
    * @param step
    * Output interval of the probe
    */
   private void createProbeAndPanel (
      ModelComponent comp, ControlPanel panel, String prop, double start,
      double stop, double step) {
      String filepath =
         PathFinder.getSourceRelativePath (
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
      Double step = getMaxStepSize ();
      VelocityInputProbe velProbe =
         VelocityInputProbe
            .createInterpolated ("target velocities", posProbe, step);
      // smooth velocities, since they were kind of wobbly. Done by John,
      // generalised by me
      int wsize = (int)(0.21 / getMaxStepSize ());
      velProbe.smoothWithSavitzkyGolay (wsize, 4);
      velProbe.setActive (true);
      System.out.println (
            "Generated velocity input probes based on marker trajectories");
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
      // Check for unclosed meshes
      //myBodies.forEach (b -> {
      //   PolygonalMesh mesh = b.getCollisionMesh ();
      //   if (!mesh.isClosed ()) {
      //      System.out.print (b.getName () + ": read ");
      //      System.out.print (mesh.getFaces ().size () + " faces. ");
      //      System.out.println ("Warning: Mesh not closed.");
      //   }
      //});
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
      RenderableComponentList<RigidBody> myBodies) {
      double comp = 0;
      RenderableComponentList<JointBase> joints;
      if (myMech.contains (myMech.get ("jointset"))) {
         System.out.println ("Generated joint constraints from jointset.");
         joints = getJointsFromJointset (comp);
      }
      else {
         System.out
            .println (
               "Generated joint constraints from ridig body connectors.");
         joints = getJointsFromBodyset (comp);
      }
      //ComponentUtils
      //   .deleteComponentAndDependencies (
      //      myMech.findComponent ("jointset/ground_pelvis"));
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
    * @param compMagnitude
    * compliance magnitude
    * @return list of {@link OpenSimCustomJoint}
    */
   private RenderableComponentList<JointBase> getJointsFromBodyset (
      double compMagnitude) {
      // Ground shares no joint with any rigid body else than the hip
      // so skip that, since hip is going to be addressed either way.
      myBodies.forEach (rb -> {
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
                  range.set (0, 0);
                  myJoints.get (end - 1).setCoordinateRangeDeg (0, range);
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
                  range.set (0, 0);
                  myJoints.get (end - 1).setCoordinateRangeDeg (0, range);
                  myJoints.get (end - 1).setCoordinateRangeDeg (1, range);
                  break;
               case "r_scapulothoracic":
                  range.set (0, 0);
                  myJoints.get (end - 1).setCoordinateRangeDeg (0, range);
                  break;
               case "r_sternoclavicular":
                  range.set (0, 0);
                  myJoints.get (end - 1).setCoordinateRangeDeg (0, range);
                  break;
               case "l_scapulothoracic":
                  range.set (0, 0);
                  myJoints.get (end - 1).setCoordinateRangeDeg (0, range);
                  break;
               case "l_sternoclavicular":
                  range.set (0, 0);
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
               range.set (-90, 90);
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
               //jt.setCoordinateRangeDeg (0, range);
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
               //jt.setCoordinateRangeDeg (0, range);
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
               range.set (0, 0);
               jt.setCoordinateRangeDeg (0, range);
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
               range.set (0, 0);
               jt.setCoordinateRangeDeg (0, range);
               jt.setCoordinateRangeDeg (1, range);
               break;
            case "r_scapulothoracic":
               range.set (0, 0);
               jt.setCoordinateRangeDeg (0, range);
               break;
            case "r_sternoclavicular":
               range.set (0, 0);
               jt.setCoordinateRangeDeg (0, range);
               break;
            case "l_scapulothoracic":
               range.set (0, 0);
               jt.setCoordinateRangeDeg (0, range);
               break;
            case "l_sternoclavicular":
               range.set (0, 0);
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
      System.out.println ("Model markers: " + myMarkers.size ());
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
               "Warning: Attachment adjusted for " + m.getName () + " from "
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
    * Queries all {@link MultiPointMuscle} objects from the current
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
         frc.getChildren ().forEachRemaining (obj -> {
            if (obj instanceof PointList)
               return;
            if (obj instanceof MuscleComponent) {
               MuscleComponent muscle = (MuscleComponent)obj;
               muscle.setExcitation (0.0);
               myMuscles.add (muscle);
            }
         });
      });
      setSimpleMuscles (problemMuscles);
      //setMillardMuscles(problemMuscles);
      setSimpleMuscles (gluts);
      //setMillardMuscles (gluts);
      setSimpleMuscles (pelvisFemur);
      //setMillardMuscles (pelvisFemur);
      setSimpleMuscles (pelvisTibia);
      //setMillardMuscles (pelvisTibia);
      setSimpleMuscles (femurTibia);
      //setMillardMuscles (femurTibia);
      setSimpleMuscles (femurCalcn);
      //setMillardMuscles (femurCalcn);
      setSimpleMuscles (tibiaCalcn);
      //setMillardMuscles (tibiaCalcn);
      setSimpleMuscles (tibiaToes);
      //setMillardMuscles (tibiaToes);
      setSimpleMuscles (pelvisTorso);
      //setMillardMuscles (pelvisTorso);
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
      // Inverse control
      TrackingController controller = addControllerAndProps ();
      addExcitersToController (controller);
      addPointTargetsAndProbes (controller, myMap, myMotion, start, stop);
      addForceInputProbes (myForces, start, stop, scale);
      // Parametric control
      //addCoordsInputProbes (myCoords, start, stop);
      // Add output probes
      // TODO: Numeric Monitor Probes for later mesh evaluation (for cases,
      // where the data is not simply collected but generated by a function
      // within the probe itself
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
      String side = "right";
      int num = 1;
      MOTReader motReader = new MOTReader (motFile, side, num);
      motReader.readData ();
      // Print reading details to the console
      System.out
         .println (
            "Experimental force data: " + motReader.getNumForceLabels ());
      System.out
         .println (
            "MOT file: read " + motReader.getNumForceFrames () + " frames");
      return motReader.getForceData ();
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
         if (markers.length == 1) {
            System.out
               .println (
                  "Warning: Marker " + markers[0]
                  + ": No corresponding experimental marker detected.");
            modelLabels.add (markers[0]);
            expLabels.add (null);
            weights.add (null);
         }
         else {
            modelLabels.add (markers[0]);
            expLabels.add (markers[1]);
            weights.add (Double.parseDouble (markers[2]));
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
         OpenSimParser parser = new OpenSimParser (osimFile);
         parser.setGeometryPath (geometryFile);
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
      System.out
         .println (
            "Experimental markers: " + trcReader.getMarkerLabels ().size ());
      System.out
         .println ("TRC file: read " + trcReader.getNumFrames () + " frames");
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
      // Add contact interfaces
      // RigidBody bodyA = (RigidBody)jt.getBodyA ();
      // RigidBody bodyB = (RigidBody)jt.getBodyB ();
      // Calculate compliant contact properties
      // double comp = 1;
      // double mass = bodyA.getMass () + bodyB.getMass ();
      // double damp = 2 * 1 * Math.sqrt (1 / comp * mass);
      // createCollision (bodyA, bodyB, comp, damp);
      // Initialize the contact monitor to handle all individual collision
      // responses
      String msgName = name + "/Output/" + name + "_message_file.txt";
      String msgPath =
         ArtisynthPath.getSrcRelativePath (this, msgName).toString ();
      ContactMonitor contMonitor =
         new ContactMonitor (coll.responses (), msgPath);
      contMonitor.setName ("Contact monitor");
      addMonitor (contMonitor);
   }

   /**
    * Initializes the default output probes of the inverse controller (tracked
    * positions, source positions and computed excitations) and adjusts the save
    * path.
    * 
    * @param controller
    * {@link TrackingController}
    * @param start
    * start time
    * @param stop
    * stop time
    * 
    */
   private void setDefaultOutputProbes (
      TrackingController controller, double start, double stop) {
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
    * Takes a list of muscle names and resets their material to a millard
    * muscle, which is based on the thelen muscle model.
    * 
    * @param names
    * String array containing muscle names
    * @return MuscleComponent with millard muscle material
    */
   private void setMillardMuscles (String[] names) {
      setMillardMuscles (getMuscles (names));
   }
   
   /**
    * Performs muscle material conversion, if the original material is of type
    * {@link EquilibriumAxialMuscle}, e.g. hill-type based muscle models.
    * 
    * @param muscles
    * List of MuscleComponents
    */
   private void setMillardMuscles (ArrayList<MuscleComponent> muscles) {
      for (MuscleComponent mc : muscles) {
         AxialMaterial mat = mc.getMaterial ();
         if (mat instanceof EquilibriumAxialMuscle) {
            EquilibriumAxialMuscle emat = (EquilibriumAxialMuscle)mat;
            Millard2012AxialMuscle mmat =
               new Millard2012AxialMuscle (
                  emat.getMaxIsoForce (), emat.getOptFibreLength (),
                  emat.getTendonSlackLength (), emat.getOptPennationAngle ());
            mc.setMaterial (mmat);
         }
      }
   }

   /**
    * Takes a list of muscle names and resets their material to a simple axial
    * muscle.
    * 
    * @param names
    * String array containing muscle names
    * @return MuscleComponent with simple axial material
    */
   private void setSimpleMuscles (String[] names) {
      setSimpleMuscles (getMuscles (names));
   }

   /**
    * Performs muscle material conversion, if the original material is of type
    * {@link EquilibriumAxialMuscle}, e.g. hill-type based muscle models. The
    * maximum force is 3*isometric force.
    * 
    * @param muscles
    * List of MuscleComponents
    */
   private void setSimpleMuscles (List<MuscleComponent> muscles) {
      for (MuscleComponent mc : muscles) {
         AxialMaterial mat = mc.getMaterial ();
         if (mat instanceof EquilibriumAxialMuscle) {
            EquilibriumAxialMuscle emat = (EquilibriumAxialMuscle)mat;
            mc.setMaterial (
                  new SimpleAxialMuscle (0, 0, 3 * emat.getMaxIsoForce ()));
         }
      }
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
      solver.setIntegrator (Integrator.ConstrainedBackwardEuler);
      solver.setStabilization (PosStabilization.GlobalStiffness);
      // solver.setMaxIterations (100);
      // solver.setTolerance (1e-5);
      setMaxStepSize (1e-2);
      // setAdaptiveStepping (true);
      // Damping properties
      myMech.setInertialDamping (3.0);
      // Define scale (mm = 1000, or m = 1)
      myScale = 1.0;
      myMech.setGravity (new Vector3d (0, -9.81, 0));
      if (myMech.getGravity ().equals (new Vector3d (0, 0, 0))) {
         JFrame frame = new JFrame ("Warning");
         frame.add (new JLabel ("Warning: Zero gravitation!", JLabel.CENTER));
         frame.setSize (300, 100);
         frame.setLocationRelativeTo (getMainFrame ());
         // Visibility always at last
         frame.setVisible (true);
      }
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
      output
         .append ("\n%%MODELBASE%%\n")
         .append (
            "%%------------------------------------------------------------%%\n")
         .append ("\nBODIES\n").append ("Model: ").append (path).append ("\n")
         .append ("Number of bodies: ").append (bodies.size ()).append ("\n")
         .append ("Total model mass: ")
         .append (String.format ("%.1f", myMech.getActiveMass ()))
         .append (" kg\n\n");
      // Identify unclosed meshes before appending individual info about each
      // body
      bodies.forEach (rb -> {
         if (!rb.getCollisionMesh ().isClosed ()) {
            output
               .append ("Unclosed Mesh found: ").append (rb.getName ())
               .append ("\n");
         }
      });
      output.append ("\n");
      // append individual info for each body
      bodies.forEach (rb -> {
         if (rb.getName ().equals ("ground")) 
            return;
         output
            .append (rb.getName () + "\t")
            .append (String.format ("%.3f", rb.getMass ())).append ("\tkg\n");
         ArrayList<Vertex3d> vertices = rb.getSurfaceMesh ().getVertices ();
         vertices.forEach (vt -> {
            output
               .append (vt.getIndex ()).append ("\t")
               .append (vt.getPosition ().toString ("%.3f")).append ("\n");
         });
         ArrayList<Face> faces = rb.getSurfaceMesh ().getFaces ();
         faces.forEach (f -> {
            output
               .append (f.idx).append ("\t")
               .append (f.getVertex (0).getIndex ()).append ("\t")
               .append (f.getVertex (1).getIndex ()).append ("\t")
               .append (f.getVertex (2).getIndex ()).append ("\t")
               .append ("\n");
         });
         output.append ("\n");
      });
      // In case it is desired to export the Mesh to a separate file.
      // String inRBMeshString = ArtisynthPath.getSrcRelativePath
      // (this,"/Input Files/" + myName + "_" +rb.getName () +
      // "_mesh.obj");
      // rb.getSurfaceMesh().print (inRBMeshString);
      // Get all vertices and faces of each rigid body and print its
      // index and parameters
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
               output
                  .append ("Point ").append (i).append ("\t")
                  .append (multi.getPoint (i).getPosition ().toString ("%.3f"))
                  .append ("\n");
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
      StringBuilder output, TrackingController controller,
      MarkerMotionData motion, MarkerMapping map, ForceData forces,
      RenderableComponentList<FrameMarker> marker) {
      output
         .append ("\n%%PROBES%%\n")
         .append (
            "%%------------------------------------------------------------%%\n")
         .append ("\nTRACKING CONTROLLER INFORMATION\n")
         .append ("Performed IK at startup: ").append (myUseIK).append ("\n")
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
      output
         .append ("\nDAMPING TERM INFORMATION\n").append ("Enabled: ")
         .append (dTerm.isEnabled ()).append ("\n").append ("Weight: ")
         .append (dTerm.getWeight ()).append ("\n");
      // L2Regularization Term
      L2RegularizationTerm lTerm = controller.getL2RegularizationTerm ();
      output
         .append ("\nL2REGULARIZATION TERM INFORMATION\n").append ("Enabled: ")
         .append (lTerm.isEnabled ()).append ("\n").append ("Weight: ")
         .append (lTerm.getWeight ()).append ("\n");
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
      String header =
         String
            .format (
               "%-16s%-16s%-16s%-6s", "Model marker", "Exp. marker",
               "Attachment", "Weight");
      output.append (header).append ("\n");
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
                        "%-16s%-16s%-16s%.1f%n", mkr.getName (), label,
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
      header = String.format ("%-32s%-22s%-8s", "Name", "Type", "Enabled");
      output.append (header).append ("\n");
      for (Probe probe : iProbes) {
         String appendix =
            String
               .format (
                  "%-32s%-22s%-6s%n", probe.getName (),
                  probe.getClass ().getSimpleName (), probe.isActive ());
         output.append (appendix);
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