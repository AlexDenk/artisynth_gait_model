package artisynth.models.JW_knee_model_net_30RND_01;

import java.lang.Math;
import static java.lang.Math.sqrt;
import java.awt.Color;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.Collections;
import java.util.Random;

import artisynth.core.femmodels.AnsysCdbReader;
import artisynth.core.femmodels.FemMeshComp;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.fields.ScalarNodalField;
import artisynth.core.femmodels.FemModel.Ranging;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.gui.ControlPanel;
import artisynth.core.inverse.FrameExciter;
import artisynth.core.inverse.InverseManager;
import artisynth.core.inverse.PointExciter;
import artisynth.core.inverse.TargetPoint;
import artisynth.core.inverse.TrackingController;
import artisynth.core.inverse.FrameExciter.WrenchDof;
import artisynth.core.inverse.InverseManager.ProbeID;
import artisynth.core.inverse.PointExciter.ForceDof;
import artisynth.core.materials.Blankevoort1991AxialLigament;
import artisynth.core.materials.FemMaterial;
import artisynth.core.materials.LinearMaterial;
import artisynth.core.materials.SimpleAxialMuscle;
import artisynth.core.mechmodels.Collidable;
import artisynth.core.mechmodels.CollisionBehavior;
import artisynth.core.mechmodels.CollisionManager;
import artisynth.core.mechmodels.CollisionResponse;
import artisynth.core.mechmodels.ConnectableBody;
import artisynth.core.mechmodels.ContactData;
import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.HingeJoint;
import artisynth.core.mechmodels.JointBase;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MechSystemSolver;
import artisynth.core.mechmodels.MultiPointMuscle;
import artisynth.core.mechmodels.MultiPointSpring;
import artisynth.core.mechmodels.Particle;
import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.PointForce;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.mechmodels.Collidable.Collidability;
import artisynth.core.mechmodels.MechSystemSolver.PosStabilization;
import artisynth.core.modelbase.Controller;
import artisynth.core.modelbase.MonitorBase;
import artisynth.core.probes.DataFunction;
import artisynth.core.probes.NumericInputProbe;
import artisynth.core.probes.NumericMonitorProbe;
import artisynth.core.probes.NumericOutputProbe;
import artisynth.core.probes.TracingProbe;
import artisynth.core.renderables.ColorBar;
import artisynth.core.workspace.RootModel;
import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.interpolation.Interpolation;
import maspack.matrix.AxisAlignedRotation;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.SymmetricMatrix3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;
import maspack.render.RenderList;
import maspack.render.RenderProps;
import maspack.render.Renderer.AxisDrawStyle;
import maspack.render.Renderer.PointStyle;
import maspack.util.Clonable;
import maspack.util.DoubleInterval;
import maspack.util.PathFinder;
import artisynth.core.driver.Main;




public class JWKneeModelNet30RND01 extends RootModel {

   // Get the data path
   String myPath =
      maspack.util.PathFinder.getSourceRelativePath (this, "data/");
   // Create a new mechanical model
   MechModel myMech = new MechModel ();
   // Value for rigid body
   RigidBody Femur, TibiaFibula, Patella;
   // Value for fem model
   FemModel3d meshFemurCart, meshTibiaCart, meshPatellaCart, meshMeniscus;
   // PolygonalMesh FemurCartCollision;
   PolygonalMesh FemurCartCollision, TibiaCartCollision, PatellaCartCollision, MeniscusCollision;
   // PolygonalMesh for Intersection Check
   PolygonalMesh IntersectionFemurPatella, IntersectionFemurMeniscus, IntersectionTibiaMeniscus;
   //Value for embedded mesh
   FemMeshComp EmbeddedFemurCartCollision, EmbeddedTibiaCartCollision, EmbeddedPatellaCartCollision, EmbeddedMeniscusCollision;

   // Create a joint value between Ground and TibialFibula
   JointBase myTibiaGroundJoint;
   // Create a collision responses
   CollisionResponse myResp;
   //  store muscle references
   private List<MultiPointSpring> muscles = new ArrayList<>();
   
   private Particle achillesPoint;
   private Particle cuneiforme;

   public void build (String[] args) throws IOException {
      addModel (myMech);
      // Set a gravity
      // Units: mm/s^2

      myMech.setGravity (0, -9810, 0);
      // Intigrator Schrittgröße
      setMaxStepSize(0.00001);                           
      setMinStepSize(1e-8);                               // test? normal -7
      setAdaptiveStepping(true);
      
      MechSystemSolver solver = myMech.getSolver();
      solver.setMaxIterations(40);                        // test? normal 20
//      solver.setTolerance(1e-6);                          // test? normal -8

      addBreakPoint (0.5000);
      MechSystemSolver.setHybridSolvesEnabled (false);
      //MechSystemSolver Solver = myMech.getSolver ();
      //Solver.setStabilization(PosStabilization.GlobalStiffness);
      myMech.setStabilization (PosStabilization . GlobalStiffness );


// ####################################################################################################### for a better view     
      setDefaultViewOrientation ( AxisAlignedRotation.NZ_Y );
     
// #####################################################################################################################  
      
 
      // Import rigid bodies
      // All model dimensions are originally in mm.
      // To ensure unit consistency, all dimensions are converted to meters.
      // Bone, density in kg/mm³
      // Cartilage, density in kg/mm³
      
      Femur = importRigidBody ("femur.obj", "Femur", 1.3e-6);
      TibiaFibula = importRigidBody ("tibia.obj", "TibiaFibula", 1.3e-6);
      Patella = importRigidBody ("patella.obj", "Patella", 1.3e-6);
 
    
    FemurCartCollision = new PolygonalMesh (myPath + "FemurCartCollision_m30ppp.obj");
    TibiaCartCollision = new PolygonalMesh (myPath + "TibiaCartCollision_m30ppp.obj");
    PatellaCartCollision = new PolygonalMesh (myPath + "PatellaCartCollision_m30ppp.obj");
    MeniscusCollision = new PolygonalMesh (myPath + "MeniscusCollision_m30ppp.obj");
    
    
    
//##################################################################################### move it (to avoid Intersection ) ###################################     
    RigidTransform3d moveitup = new RigidTransform3d(0.0, 0.36, 0.0);
    FemurCartCollision.transform(moveitup );
    
    
    RigidTransform3d moveitforward = new RigidTransform3d(0.0, 0, 0.49);
    PatellaCartCollision.transform(moveitforward);
    
    
    RigidTransform3d moveitdown = new RigidTransform3d(0.0, -0.32, 0.0);
    TibiaCartCollision.transform(moveitdown);           
//####################################################################################################################################### 

    
    
    // but the mashes are not build by triangles !!! ---> mesh must be triangulated
    FemurCartCollision.triangulate ();
    TibiaCartCollision.triangulate ();
    PatellaCartCollision.triangulate ();
    MeniscusCollision.triangulate (); 


      // Import FEM-models
      
      
      
      meshFemurCart =
         importFemModel (
            "FemurCart_m30.cdb", "FemurCart", 1.15e3, // density in kg/mm³
            0.01, // mass damping
            1, // stiffness damping
            new LinearMaterial (
               /* Young's modulu(MPa): */50000000, /* Poisson's ratio: */ 0.45));
      meshFemurCart.scaleDistance(1000);

      EmbeddedFemurCartCollision = meshFemurCart.addMesh("FemurCartCollision", FemurCartCollision);
      EmbeddedFemurCartCollision.setSurfaceRendering( SurfaceRender.Shaded );
      EmbeddedFemurCartCollision.setCollidable(Collidability.EXTERNAL);
      RenderProps.setFaceColor (EmbeddedFemurCartCollision, new Color (0.8f, 0.5f, 0.6f));
      RenderProps.setAlpha (EmbeddedFemurCartCollision, 0.5);   // opaque    
      
      meshTibiaCart =
         importFemModel (
            "TibiaCart_m30.cdb", "TibiaCart", 1.15e3, // density in kg/mm³
            0.01, // mass damping
            1, // stiffness damping
            new LinearMaterial (
               /* Young's modulu(MPa): */50000000, /* Poisson's ratio: */ 0.45));
      meshTibiaCart.scaleDistance(1000);
            
      EmbeddedTibiaCartCollision = meshTibiaCart.addMesh("TibiaCartCollision", TibiaCartCollision);
      EmbeddedTibiaCartCollision.setSurfaceRendering( SurfaceRender.Shaded );
      EmbeddedTibiaCartCollision.setCollidable(Collidability.EXTERNAL);
      RenderProps.setFaceColor (EmbeddedTibiaCartCollision, new Color (0.8f, 0.5f, 0.6f));
      RenderProps.setAlpha (EmbeddedTibiaCartCollision, 0.5);   // opaque
      
      meshPatellaCart =
         importFemModel (
            "PatellaCart_m30.cdb", "PatellaCart", 1.15e3, // density in kg/mm³
            0.01, // mass damping
            1, // stiffness damping
            new LinearMaterial (
               /* Young's modulu(50 MPa): */50000000, /* Poisson's ratio: */ 0.45));
      meshPatellaCart.scaleDistance(1000);
      
      EmbeddedPatellaCartCollision = meshPatellaCart.addMesh("PatellaCartCollision", PatellaCartCollision);
      EmbeddedPatellaCartCollision.setSurfaceRendering( SurfaceRender.Shaded );
      EmbeddedPatellaCartCollision.setCollidable(Collidability.EXTERNAL);
      RenderProps.setFaceColor (EmbeddedPatellaCartCollision, new Color (0.9f, 0.4f, 0.4f));
      RenderProps.setAlpha (EmbeddedPatellaCartCollision, 0.5);   // opaque

      meshMeniscus =
         importFemModel (
            "Meniscus_m30.cdb", "Meniscus", 1.15e3, // density in kg/mm³
            0.01, // mass damping
            1, // stiffness damping
            new LinearMaterial (
               /* Young's modulu(MPa): */50000000, /* Poisson's ratio: */ 0.45));
      meshMeniscus.scaleDistance(1000);
      //RenderProps.setSphericalPoints (meshMeniscus, 0.2, Color.CYAN);
      
      
      EmbeddedMeniscusCollision = meshMeniscus.addMesh("MeniscusLatCartCollision", MeniscusCollision);
      EmbeddedMeniscusCollision.setSurfaceRendering( SurfaceRender.Shaded );
      EmbeddedMeniscusCollision.setCollidable(Collidability.EXTERNAL);
      RenderProps.setFaceColor (EmbeddedMeniscusCollision, new Color (0.8f, 0.5f, 0.6f));
      RenderProps.setAlpha (EmbeddedMeniscusCollision, 0.0);   // opaque      
      


      // Connecting the FEM-model to rigid body model with nodes of mesh model
      PolygonalMesh femurSurface = Femur.getSurfaceMesh ();
      double tol = 0.7;
      for (FemNode3d n : meshFemurCart.getNodes ()) {
         if (meshFemurCart.isSurfaceNode (n)) {
            Point3d n_Position = n.getPosition ();
            if (femurSurface.distanceToPoint (n_Position) < tol) {
                //RenderProps.setVisible(n, true);
                //RenderProps.setSphericalPoints(n, 0.1, Color.red);
               myMech.attachPoint (n, Femur);
            }
         }
      }
      
      PolygonalMesh TibiaSurface = TibiaFibula.getSurfaceMesh ();
      double tol2 = 0.3;
      for (FemNode3d n : meshTibiaCart.getNodes ()) {
         if (meshTibiaCart.isSurfaceNode (n)) {
            Point3d n_Position = n.getPosition ();
            if (TibiaSurface.distanceToPoint (n_Position) < tol2) {
                //RenderProps.setVisible(n, true);
                //RenderProps.setSphericalPoints(n, 0.1, Color.red);
               myMech.attachPoint (n, TibiaFibula);
            }
         }
      }
      
      PolygonalMesh PatellaSurface = Patella.getSurfaceMesh ();
      double tol3 = 0.8;
      for (FemNode3d n : meshPatellaCart.getNodes ()) {
         if (meshPatellaCart.isSurfaceNode (n)) {
            Point3d n_Position = n.getPosition ();
            if (PatellaSurface.distanceToPoint (n_Position) < tol3) {
                //RenderProps.setVisible(n, true);
                //RenderProps.setSphericalPoints(n, 0.1, Color.red);
               myMech.attachPoint (n, Patella);
            }
         }
      }



      
      //myTibiaGroundJoint = createJointToGround(TibiaFibula);
      myTibiaGroundJoint =createHingeJointToGround(TibiaFibula);
      // Set dynamic property of models
      Femur.setDynamic (true);
      TibiaFibula.setDynamic (true);

      
      // Set collision behaviors of models
      // Body1, Body2, mu, compliance, damping  
      // critically damping ≈ 2 * sqrt( (1/compliance) * mass) mass ist nicht defined =( willkürlich angenommen m=  1 Gramm

// test smaller compliance and pseudo crit. damping    
      setCollisionBehavior (Femur, EmbeddedPatellaCartCollision, 0, 1e-6, 2 * sqrt( (1/1e-6) * 1e-3) );
      setCollisionBehavior (Femur, Patella, 0, 1e-6, 2 * sqrt( (1/1e-6) * 1e-3)); //does it matter?
      setCollisionBehavior (EmbeddedFemurCartCollision, EmbeddedPatellaCartCollision,  0, 1e-6, 2 * sqrt( (1/1e-6) * 1e-3));
      setCollisionBehavior (EmbeddedFemurCartCollision, EmbeddedMeniscusCollision,  0, 1e-6, 2 * sqrt( (1/1e-6) * 1e-3));
      setCollisionBehavior (EmbeddedFemurCartCollision, EmbeddedTibiaCartCollision,  0, 1e-6, 2 * sqrt( (1/1e-6) * 1e-3));
      setCollisionBehavior (EmbeddedPatellaCartCollision, EmbeddedTibiaCartCollision,  0, 1e-6, 2 * sqrt( (1/1e-6) * 1e-3));
      setCollisionBehavior (EmbeddedTibiaCartCollision, EmbeddedMeniscusCollision,  0, 1e-6, 2 * sqrt( (1/1e-6) * 1e-3));
      
/*     
      setCollisionBehavior (Femur, EmbeddedPatellaCartCollision, 0, 0.3e-8, 1e8);
      setCollisionBehavior (Femur, Patella, 0, 1e-9, 1.5e8); //does it matter?
      setCollisionBehavior (EmbeddedFemurCartCollision, EmbeddedPatellaCartCollision, 0, 0.6e-9 , 1.5e8);
      setCollisionBehavior (EmbeddedFemurCartCollision, EmbeddedMeniscusCollision, 0, 0.2e-8 , 1.5e8);
      setCollisionBehavior (EmbeddedFemurCartCollision, EmbeddedTibiaCartCollision, 0, 0.3e-9 , 1.5e8);
      setCollisionBehavior (EmbeddedPatellaCartCollision, EmbeddedTibiaCartCollision, 0, 0.3e-9 , 1.5e8);
      setCollisionBehavior (EmbeddedTibiaCartCollision, EmbeddedMeniscusCollision, 0, 0.2e-8 , 1.5e8);
*/

      setCollisionManager ();

      
      
      // create Forces
      
      Point3d ForcePointW = new Point3d (354.53, 1815.51, 772.74);    // Gewichtskraft 343.35N  (70Kg*9,81m/(s*s)*0.5)
    //  Point3d ForcePointA = new Point3d (354.53, 1815.51, 772.74);        
    //  Point3d ForcePointB = new Point3d (-9.75, 416.98, -0.018);
      
      FrameMarker mkrW = myMech.addFrameMarker (Femur, ForcePointW); 
    //  FrameMarker mkrA = myMech.addFrameMarker (Femur, ForcePointA);
    //  FrameMarker mkrB = myMech.addFrameMarker (Femur, ForcePointB);
      
      Vector3d VectorFW = new Vector3d (0, -343350, 0);                 // 100% = 343350 mN
    //  Vector3d VectorFA = new Vector3d (-1000, 0, 0);                    // 
    //  Vector3d VectorFB = new Vector3d (50, 0, 0); 
      
      PointForce ForceVectorFW = new PointForce (VectorFW, mkrW);
    //  PointForce ForceVectorFA = new PointForce (VectorFA, mkrA);
    //  PointForce ForceVectorFB = new PointForce (VectorFB, mkrB);
      
      myMech.addForceEffector (ForceVectorFW);
    //  myMech.addForceEffector (ForceVectorFA);
    //  myMech.addForceEffector (ForceVectorFB);
      
      
 //#################################################################### Check forIntersection ###################################
   
    IntersectionFemurPatella = MeshFactory.getIntersection(FemurCartCollision,PatellaCartCollision);
    RenderProps.setVisible(IntersectionFemurPatella , true);
    RenderProps.setAlpha(IntersectionFemurPatella , 1.0);
    RenderProps.setFaceColor (IntersectionFemurPatella , Color.RED);  
    FixedMeshBody IntersectionFemurPatellabody = new FixedMeshBody("IntersectionFemurPatella", IntersectionFemurPatella);  
    myMech.addMeshBody(IntersectionFemurPatellabody);
   
    IntersectionFemurMeniscus = MeshFactory.getIntersection(FemurCartCollision,MeniscusCollision);
    RenderProps.setVisible(IntersectionFemurMeniscus , true);
    RenderProps.setAlpha(IntersectionFemurMeniscus , 1.0);
    RenderProps.setFaceColor (IntersectionFemurMeniscus , Color.RED); 
    FixedMeshBody IntersectionFemurMeniscusbody = new FixedMeshBody("IntersectionFemurMeniscus", IntersectionFemurMeniscus);  
    myMech.addMeshBody(IntersectionFemurMeniscusbody); 
       
    IntersectionTibiaMeniscus = MeshFactory.getIntersection(TibiaCartCollision, MeniscusCollision);
    RenderProps.setVisible(IntersectionTibiaMeniscus , true);
    RenderProps.setAlpha(IntersectionTibiaMeniscus , 1.0);
    RenderProps.setFaceColor (IntersectionTibiaMeniscus , Color.RED); 
    FixedMeshBody IntersectionTibiaMeniscusbody = new FixedMeshBody("IntersectionTibiaMeniscus", IntersectionTibiaMeniscus);  
    myMech.addMeshBody(IntersectionTibiaMeniscusbody);     
    //######################################################################################## Marker points Calcaneus + Os cuneiforme ##########################
    //Create a fixed point in space for Achillesntendon to attach
  
    cuneiforme = new Particle("Cuneiforme", 0, 390.0, 965.0, 850.0);
    cuneiforme.setName("MTA_insertion");
    myMech.addParticle(cuneiforme);
    RenderProps.setSphericalPoints(cuneiforme, 2.0, Color.YELLOW);
    cuneiforme.setDynamic(false);
  
    
//  #####################################################################################  white background  ##########################
/*   PolygonalMesh planeMesh = MeshFactory.createPlane(4000, 4000, 1, 1);
    FixedMeshBody backgroundPlane = new FixedMeshBody("whiteBackground", planeMesh);
    myMech.addMeshBody(backgroundPlane);
    RotationMatrix3d rot = new RotationMatrix3d();
    rot.setRpy(0, 0, Math.toRadians(-90)); // (roll, pitch, yaw)
    Vector3d pos = new Vector3d(0, -1000, 0); 
    backgroundPlane.setPose(new RigidTransform3d(pos, rot));
    RenderProps.setFaceColor(backgroundPlane, Color.WHITE);
    RenderProps.setFaceStyle(backgroundPlane, FaceStyle.FRONT);
    backgroundPlane.setSelectable(false);
    */   
//  ###################################################################################################################################
      // Add the color bar
      createColorBar ();

      // Initialize writerfile to record Displacement, Stress, Strain of nodes
      initialWriter ();

      // Add a probe to monitor or collect specific data during the simulation
      addProbe ();

      // Set a stop time of the simulation
      addBreakPoint (4.0);
          
      // Import ligaments
      addAnterolateralLigament();
      addAnteriorCruciateLigament();
      addLateralCollateralLigament();
      addMedialCollateralLigamentComplex();
      addPosteriorCruciateLigament();
      addPopliteusObliqueLigament();
      addPatellarLigaments();
      
      addAchillesTendon();
      
      // Import Muscle
      addBicepsFemorisMuscle(); 
      addSemitendinosusMuscle();
      addSoleusMuscle(); 
      addGastrocnemiusMuscles();
      addTibialisAnteriorMuscle();
      addMuscle (Femur, Patella);
      
//########################################################################################### statistics ###############################
      printFemModelInfo(meshMeniscus, "Meniscus");
      printFemModelInfo(meshFemurCart, "Femur Cartilage");
      printFemModelInfo(meshTibiaCart, "Tibia Cartilage");
      printFemModelInfo(meshPatellaCart, "Patella Cartilage");
//######################################################################################################################################
      
      
    // Add camera view for simulation
      CameraView();
      
//########################################################################################  visibility block  ##########################
      
      RenderProps.setVisible(meshMeniscus, true);
      RenderProps.setVisible(Femur, true);
      RenderProps.setVisible(TibiaFibula, true);
      RenderProps.setVisible(Patella, true);
      RenderProps.setVisible(meshFemurCart, true);
      RenderProps.setVisible(meshTibiaCart, true);
      RenderProps.setVisible(meshPatellaCart, true);
      
      //RenderProps.setAlpha(meshMeniscus, 0.8);
      
//##############################################################################################   Meniscus Ligament   ####################

   // Transverse Ligament: n10,n5955   379.65167 1358.1027 862.36358 and 345.50379 1358.9717 854.44787 d= 35.06 
   FemNode3d MTL1 = meshMeniscus.getNode(10);
   FemNode3d MTL2 = meshMeniscus.getNode(5955);
   MultiPointSpring MTL = new MultiPointSpring("Meniscus Transverse Ligament");
   MTL.addPoint(MTL1);
   MTL.addPoint(MTL2);
   MTL.setMaterial(new Blankevoort1991AxialLigament(20000, 34 , 0));
   myMech.addMultiPointSpring(MTL);
   RenderProps.setCylindricalLines(MTL, 0.3, Color.WHITE);
   RenderProps.setSphericalPoints(MTL1, 0.4, Color.WHITE);
   RenderProps.setSphericalPoints(MTL2, 0.4, Color.WHITE);

 // Medial Meniscal Anterior Root Ligament   n7,n2946 380.17184 1356.9142 863.49374  and  367.81818 1351.5483 867.25482 (tibia) d=13.99  
   FemNode3d MMARL1 = meshMeniscus.getNode(7);
   Point3d MMARLPointOnBody = new Point3d(367.81818, 1351.5483, 867.25482); 
   FrameMarker MMARL2 = new FrameMarker();
   myMech.addFrameMarker(MMARL2, TibiaFibula, MMARLPointOnBody);
   MultiPointSpring MMARL = new MultiPointSpring("Medial Meniscal Anterior Root Ligament");
   MMARL.addPoint(MMARL1);
   MMARL.addPoint(MMARL2);
   MMARL.setMaterial(new Blankevoort1991AxialLigament(20000, 12 , 0));
   myMech.addMultiPointSpring(MMARL);
   RenderProps.setCylindricalLines(MMARL, 0.3, Color.WHITE);
   RenderProps.setSphericalPoints(MMARL1, 0.4, Color.WHITE);
   RenderProps.setSphericalPoints(MMARL2, 0.4, Color.WHITE);  

   // Medial Meniscal Posterior Root Ligament  n76, n8766  372.92001 1361.5133 827.97372 and 357.3945 1360.1871 833.15985 (tibia) d=16.42
   FemNode3d MMPRL1 = meshMeniscus.getNode(76);
   Point3d MMPRLPointOnBody = new Point3d(357.3945, 1360.1871, 833.15985); 
   FrameMarker MMPRL2 = new FrameMarker();
   myMech.addFrameMarker(MMPRL2, TibiaFibula, MMPRLPointOnBody);
   MultiPointSpring MMPRL = new MultiPointSpring("Medial Meniscal Posterior Root Ligament");
   MMPRL.addPoint(MMPRL1);
   MMPRL.addPoint(MMPRL2);
   MMPRL.setMaterial(new Blankevoort1991AxialLigament(10000, 15.4 , 0));
   myMech.addMultiPointSpring(MMPRL);
   RenderProps.setCylindricalLines(MMPRL, 0.3, Color.WHITE);
   RenderProps.setSphericalPoints(MMPRL1, 0.4, Color.WHITE);
   RenderProps.setSphericalPoints(MMPRL2, 0.4, Color.WHITE);     
 
   // Lateral Meniscal Posterior Root Ligament  n5723, n8842  350.76427 1362.102 826.95961 and 363.44931 1361.9719 836.72845 d=16.01
   FemNode3d LMPRL1 = meshMeniscus.getNode(5723);
   Point3d LMPRLPointOnBody = new Point3d(363.44931, 1361.9719, 836.72845); 
   FrameMarker LMPRL2 = new FrameMarker();
   myMech.addFrameMarker(LMPRL2, TibiaFibula, LMPRLPointOnBody);
   MultiPointSpring LMPRL = new MultiPointSpring("Lateral Meniscal Posterior Root Ligament");
   LMPRL.addPoint(LMPRL1);
   LMPRL.addPoint(LMPRL2);
   LMPRL.setMaterial(new Blankevoort1991AxialLigament(10000, 15.01 , 0));
   myMech.addMultiPointSpring(LMPRL);
   RenderProps.setCylindricalLines(LMPRL, 0.3, Color.WHITE);
   RenderProps.setSphericalPoints(LMPRL1, 0.4, Color.WHITE);
   RenderProps.setSphericalPoints(LMPRL2, 0.4, Color.WHITE);    
   
   // Lateral Meniscal Anterior Root Ligament  n5106,n8841  346.37848 1359.6295 850.64185  and 364.20004 1361.462 845.92108 d=18.52
   FemNode3d LMARL1 = meshMeniscus.getNode(5106);
   Point3d LMARLPointOnBody = new Point3d(364.20004, 1361.462, 845.92108); 
   FrameMarker LMARL2 = new FrameMarker();
   myMech.addFrameMarker(LMARL2, TibiaFibula, LMARLPointOnBody);
   MultiPointSpring LMARL = new MultiPointSpring("Lateral Meniscal Anterior Root Ligament");
   LMARL.addPoint(LMARL1);
   LMARL.addPoint(LMARL2);
   LMARL.setMaterial(new Blankevoort1991AxialLigament(10000, 17.52 , 0));
   myMech.addMultiPointSpring(LMARL);
   RenderProps.setCylindricalLines(LMARL, 0.3, Color.WHITE);
   RenderProps.setSphericalPoints(LMARL1, 0.4, Color.WHITE);
   RenderProps.setSphericalPoints(LMARL2, 0.4, Color.WHITE);  
   
   //Posterior MeniscoFemoral Ligament n5725, n636  349.66889 1365.8246 823.37004 and 369.18301 1377.5077 842.44153 (femur) d=29.68
   FemNode3d PMFL1 = meshMeniscus.getNode(5725);
   Point3d PMFLPointOnBody = new Point3d( 369.18301, 1377.5077, 842.44153); 
   FrameMarker PMFL2 = new FrameMarker();
   myMech.addFrameMarker(PMFL2, TibiaFibula, PMFLPointOnBody);
   MultiPointSpring PMFL = new MultiPointSpring("Posterior MeniscoFemoral Ligament");
   PMFL.addPoint(PMFL1);
   PMFL.addPoint(PMFL2);
   PMFL.setMaterial(new Blankevoort1991AxialLigament(10000, 28.68 , 0));
   myMech.addMultiPointSpring(PMFL);
   RenderProps.setCylindricalLines(PMFL, 0.3, Color.WHITE);
   RenderProps.setSphericalPoints(PMFL1, 0.4, Color.WHITE);
   RenderProps.setSphericalPoints(PMFL2, 0.4, Color.WHITE);  
   
   //Anterior MeniscoFemoral Ligament n5736, n3107 350.65588 1364.8883 825.89853 and 364.12494 1374.2969 851.49683(femur) d=30.41
   FemNode3d AMFL1 = meshMeniscus.getNode(5736);
   Point3d AMFLPointOnBody = new Point3d(364.12494, 1374.2969, 851.49683); 
   FrameMarker AMFL2 = new FrameMarker();
   myMech.addFrameMarker(AMFL2, TibiaFibula, AMFLPointOnBody);
   MultiPointSpring AMFL = new MultiPointSpring("Anterior MeniscoFemoral Ligament");
   AMFL.addPoint(AMFL1);
   AMFL.addPoint(AMFL2);
   AMFL.setMaterial(new Blankevoort1991AxialLigament(10000, 29.41 , 0));
   myMech.addMultiPointSpring(AMFL);
   RenderProps.setCylindricalLines(AMFL, 0.3, Color.WHITE);
   RenderProps.setSphericalPoints(AMFL1, 0.4, Color.WHITE);
   RenderProps.setSphericalPoints(AMFL2, 0.4, Color.WHITE); 
   

   
// ###########################################################################################   Setting up the tracking controller  ##### 
   FrameMarker Target_Mkr  = myMech.addFrameMarkerWorld (Femur, new Point3d(378.60, 1789.80, 781.02));  // 378.59409 1789.7974 781.01996
   RenderProps.setSphericalPoints(Target_Mkr, 1.5, Color.WHITE);
   
   double startTime = 0; // probe start times
   double stopTime = 3.3;  // probe stop times
   
   double x0 = 364.72946;      // initial x coordinate of the Hip marker for point PointExciter   364.72946 1816.1625 783.08197
   double y0 = 1816.1625;      // initial y coordinate of the Hip marker for point PointExciter
   double z0 = 783.08197;      // initial z coordinate of the Hip marker for point PointExciter

   double kx0 = 359.93979;     // initial x coordinate of the knee marker for point PointExciter     359.93979 1377.0413 849.25525
   double ky0 = 1377.0413;     // initial y coordinate of the knee marker for point PointExciter
   double kz0 = 849.25525;     // initial z coordinate of the knee marker for point PointExciter

   // Create points using FrameMarkers 
   FrameMarker pH = myMech.addFrameMarker(Femur, new Point3d(x0, y0, z0));
   pH.setName("Hip_Point");
   FrameMarker pK = myMech.addFrameMarker(Femur, new Point3d(kx0, ky0, kz0));  
   pK.setName("Knee_Point");

   
   // create the tracking controller and add it to the root model
   TrackingController tcon = new TrackingController(myMech, "tcon");
   addController(tcon);  
   
   tcon.setComputeIncrementally (true);  // very important!
   // the target:
   TargetPoint target = tcon.addPointTarget(Target_Mkr);
   
   String myCSVPath = maspack.util.PathFinder.getSourceRelativePath(this, "data/H_Target_Position_MA.txt");
   NumericInputProbe targetprobe = InverseManager.createInputProbe (
      tcon, ProbeID.TARGET_POSITIONS, /*fileName=*/myCSVPath,
      startTime, stopTime);
   
   targetprobe.setInterpolationOrder (Interpolation.Order.Cubic);
   
   addInputProbe (targetprobe);
   
   tcon.setL2Regularization(/*weight=*/0.05); 
   
   // Add exciters to the FrameMarkers instead of Points
   addPointExciter(tcon, pH, ForceDof.FX, 1000, 0.1, "HX");  
   addPointExciter(tcon, pH, ForceDof.FY, 1000, 0.1, "HY");
   addPointExciter(tcon, pH, ForceDof.FZ, 1000000, 0.2, "HZ");  // this one prevents tipping over to the side
   addPointExciter(tcon, pK, ForceDof.FX, 1000, 0.1, "KX");
   addPointExciter(tcon, pK, ForceDof.FY, 1000, 0.1, "KY");
   
   // translational forces in the x-y plane              
   addFrameExciter (tcon, Femur, WrenchDof.FZ, 1000, 0.05, "FemurFrameZ");
   addFrameExciter (tcon, Femur, WrenchDof.FX, 1000, 0.05, "FemurFrameX");
   addFrameExciter (tcon, TibiaFibula, WrenchDof.FZ, 1000, 0.05, "TibiaFrameZ");
   addFrameExciter (tcon, TibiaFibula, WrenchDof.FX, 1000, 0.05, "TibiaFrameX");
 
   
   // sets all muscles to be Exciters 
   for (MultiPointSpring s : myMech.multiPointSprings()) {
      if (s instanceof MultiPointMuscle) {
         tcon.addExciter(0.4,(MultiPointMuscle)s);
      }
   }
     
   // tcon.initializeExcitations(); not important
   
   // add an output probe to record the excitations:
   NumericOutputProbe exprobe = InverseManager.createOutputProbe (
      tcon, ProbeID.COMPUTED_EXCITATIONS, /*fileName=*/null,
      startTime, stopTime, /*interval=*/-1);
   addOutputProbe (exprobe);

   
   // add tracing probes to view the tracking target (cyan) and the actual tracked position (red).
   
   TracingProbe tprobe;
   tprobe = addTracingProbe (target, "position", startTime, stopTime);      
   tprobe.setName ("target tracing");
   RenderProps.setLineColor (tprobe, Color.CYAN);      
   tprobe = addTracingProbe (Target_Mkr, "position", startTime, stopTime);
   tprobe.setName ("source tracing");
   RenderProps.setLineColor (tprobe, Color.RED);

   InverseManager.addInversePanel (this, tcon);      //     <<<<<------------------------------------------ Panel
   
   
   // geometry notes top: 357.21875 1817.0209 784.27704     bot: 390 965 850   ---> h=852 z=frontal richtung x=medial y=up
// #######################################################################################################################################
   }  //    <---- this is the END of the build() method
   
   
   
 //##############################################################################################   CAM VIEW  constr.  ####################
   
   private boolean camViewON = false;  
   // 3. GETTER:
   public boolean getCamViewON() {
    return camViewON;
   }  
   // 4. SETTER:
   public void setCamViewON(boolean on) {
    camViewON = on;    
    if (camViewON) {CameraView();}
   }

 //##############################################################################################   CAM VIEW on meniscus   ################
   
   private void CameraView() {
      //point 354.04495 1460.5951 860.86957
      Point3d center = new Point3d(366.0, 500, 844.0);
      setViewerCenter(center);

      // Set the CAMERA POSITION (the "eye")
      Point3d eye = new Point3d(366.0, 1505.0, 844.0);
      setViewerEye(eye);

      // Define which axis is "Up" (default is Z)
      Vector3d up = new Vector3d(0, -1, 0);
      setViewerUp(up);      
   }
   
   //##########################################################################   Methode for Frame + PointExciter   #######################
   void addFrameExciter (
      TrackingController tcon, RigidBody body, WrenchDof dof, double maxf, double weight, String name) {
      FrameExciter fex = new FrameExciter (name, body, dof, maxf);
      myMech.addForceEffector (fex);
      tcon.addExciter (weight,fex);
   } 
     
   void addPointExciter (
      TrackingController tcon, Point  point, ForceDof dof, double maxf, double weight, String name) {
      PointExciter pex = new PointExciter (name, point, dof, maxf);
      myMech.addForceEffector (pex);
      tcon.addExciter (weight, pex);
   } 
   //#######################################################################################################################################
   
   private RigidBody importRigidBody (
      String filename, String modelname, double density)
      throws IOException {
      PolygonalMesh mesh = new PolygonalMesh (myPath + filename);
      RigidBody body = RigidBody.createFromMesh (modelname, mesh, density, 1);
      myMech.addRigidBody (body);
      return body;
   }

   private FemModel3d importFemModel (
      String filename, String modelname, double density, double massDamping,
      double stiffnessDamping, FemMaterial material)
      throws IOException {
      FemModel3d femModel = AnsysCdbReader.read (myPath + filename);
      femModel.setName (modelname);
      femModel.setDensity (density);
      femModel.setMassDamping (massDamping);
      femModel.setStiffnessDamping (stiffnessDamping);
      femModel.setMaterial (material);
      myMech.addModel (femModel);
      setFemRenderProps (femModel);
      femModel.setComputeNodalStress (true);
      femModel.setComputeNodalStrain (true);
      return femModel;
   }

   private void setFemRenderProps (FemModel3d fem) {
      // fem.setSurfaceRendering(SurfaceRender.Shaded);
      fem.setSurfaceRendering (SurfaceRender.Stress);
      //fem.setStressPlotRanging (Ranging.Auto);
      fem.setSurfaceRendering(SurfaceRender.Stress);
      fem.setStressPlotRanging(Ranging.Fixed);
      fem.setStressPlotRange(new DoubleInterval(0.0, 100000.0)); 
          
      // RenderProps.setVisible(fem.getNodes(), false);
      // RenderProps.setVisible(fem.getElements(), false);
      // RenderProps.setAlpha(fem, 1.0);
      // RenderProps.setFaceColor (fem, Color.GRAY);
      // RenderProps.setLineColor(fem, Color.DARK_GRAY);
      //RenderProps.setSphericalPoints (fem, 0.0001, Color.CYAN);
   }

   private double minValue;
   private double maxValue;

   public ColorBar createColorBar () {
      ColorBar cbar = new ColorBar ();
      cbar.setName ("colorBar");
      cbar.setNumberFormat ("%.2f");
      cbar.populateLabels (0.0, 0.1, 10);
      cbar.setLocation (-100, 0.1, 20, 0.8);
      addRenderable (cbar);
      return cbar;
   }

   public void prerender (RenderList list) {
      // Add a Scalar field
      // addScalarField();
      // Update cbar values
      super.prerender (list);
      ColorBar cbar = (ColorBar)(renderables ().get ("colorBar"));
      List<FemModel3d> femModels = Arrays.asList (meshFemurCart);
      for (FemModel3d fem : femModels) {
         cbar.setColorMap (fem.getColorMap ());
         DoubleInterval range = fem.getStressPlotRange ();
         cbar.updateLabels (range.getLowerBound (), range.getUpperBound ());
         // cbar.updateLabels(minValue, maxValue);
      }
   }
/*
   private void addScalarField () {
      ScalarNodalField field = new ScalarNodalField (meshFemurCart);
      meshFemurCart.addField (field);
      for (FemNode3d n : meshFemurCart.getNodes ()) {
         if (meshFemurCart.isSurfaceNode (n)) {
            double value = n.getMAPStress ();
            if (value < minValue) {
               minValue = value;
            }
            if (value > maxValue) {
               maxValue = value;
            }
            field.setValue (n, value);
         }
      }
      field.setVisualization (ScalarNodalField.Visualization.POINT);
      RenderProps.setPointRadius (field, 0.5);
      RenderProps.setPointStyle (field, PointStyle.SPHERE);
   }
   
   */

 // ############################################################################################## Ligma #################
   

   
// Anterolateral Ligament (ALL)
private void addAnterolateralLigament() {
    createLigament("ALL",
        Femur, new Point3d(325, 1377, 842),
        TibiaFibula, new Point3d(324, 1352, 844),
        795000, 15, 0.00);
}

//  Anterior Cruciate Ligament (ACL) - Two Bundles
private void addAnteriorCruciateLigament() {
    // Anterior bundle
    createLigament("aACL",
        Femur, new Point3d(356, 1384, 834),
        TibiaFibula, new Point3d(362, 1357, 853),
        6200000, 32.3, 0.00);
    
    // Posterior bundle
    createLigament("pACL",
        Femur, new Point3d(356, 1379, 832),
        TibiaFibula, new Point3d(358, 1357, 850),
        3400000, 26.6, 0.00);
}

//  Lateral Collateral Ligament (LCL) Complex
private void addLateralCollateralLigament() {
    // Anterior part
    createLigament("aLCL",
        Femur, new Point3d(326, 1381, 841),
        TibiaFibula, new Point3d(311, 1328, 829),
        2000000, 45, 0.00);
    
    // Middle part
    createLigament("mLCL",
        Femur, new Point3d(327, 1381, 837),
        TibiaFibula, new Point3d(311, 1328, 825),
        2000000, 40.2, 0.00);
    
    // Posterior part
    createLigament("pLCL",
        Femur, new Point3d(329, 1380, 833),
        TibiaFibula, new Point3d(313, 1331, 823),
        2000000, 40.2, 10.00);
}

//  Medial Collateral Ligament (MCL) Complex
private void addMedialCollateralLigamentComplex() {
    // Deep MCL - Anterior part
    createLigament("adMCL",
        Femur, new Point3d(398, 1381, 847),
        TibiaFibula, new Point3d(394, 1349, 848),
        1500000, 27.2, 10.00);
    
    // Deep MCL - Posterior part
    createLigament("pdMCL",
        Femur, new Point3d(398, 1382, 838),
        TibiaFibula, new Point3d(395, 1348, 838),
        1500000, 23.8, 10.00);
    
    // Superficial MCL - Anterior part (3-point ligament)
    createLigament("asMCL",
        Femur, new Point3d(398, 1384, 849),
        TibiaFibula, new Point3d(375, 1301, 851),
        new Point3d(395, 1345, 850), // Via point
        2500000, 40.3, 10.00);
    
    // Superficial MCL - Middle part (3-point ligament)
    createLigament("msMCL",
        Femur, new Point3d(399, 1385, 845),
        TibiaFibula, new Point3d(376, 1297, 848),
        new Point3d(395, 1345, 847), // Via point
        2600000, 38.6, 10.0);
    
    // Superficial MCL - Posterior part (3-point ligament)
    createLigament("psMCL",
        Femur, new Point3d(399, 1384, 840),
        TibiaFibula, new Point3d(376, 1293, 846),
        new Point3d(396, 1345, 844), // Via point
        2700000, 37.1, 10.0);
}

// Posterior Cruciate Ligament (PCL) - Two Bundles
private void addPosteriorCruciateLigament() {
    // Anterior bundle
    createLigament("aPCL",
        Femur, new Point3d(365, 1377, 849),
        TibiaFibula, new Point3d(360, 1354, 828),
        12500000, 39.7, 1.0);
    
    // Posterior bundle
    createLigament("pPCL",
        Femur, new Point3d(370, 1374, 842),
        TibiaFibula, new Point3d(362, 1350, 825),
        1500000, 38.4, 1.0);
}

// Popliteus Oblique Ligament (POL)
private void addPopliteusObliqueLigament() {
    createLigament("POL",
        Femur, new Point3d(399, 1388, 836),
        TibiaFibula, new Point3d(393, 1343, 828),
        1600000, 44.8, 1.0);
}

// Patellar Ligaments (Kges=1000 N/mm)
private void addPatellarLigaments() {
    // Lateral patellar ligament
    createLigament("lPL",
        Patella, new Point3d(347, 1382, 895),
        TibiaFibula, new Point3d(350, 1326, 869),
        33300000, 55.8, 10.0);
    
    // Central patellar ligament
    createLigament("cPL",
        Patella, new Point3d(356, 1373, 894),
        TibiaFibula, new Point3d(359, 1328, 871),
        33300000, 45.6, 10.0);
    
    // Medial patellar ligament
    createLigament("mPL",
        Patella, new Point3d(364, 1381, 897),
        TibiaFibula, new Point3d(364, 1331, 870),
        33300000, 50.0, 10.0);
    
    // 374.33295 1400.506 898.52448 patella m   397.57446 1398.1603 840.43719 Femur m
    // 334.75906 1401.676 890.80017 patella l   328.79553 1393.6785 836.6507 Femur L
    
    //  Medial patellofemoral ligament
    createLigament("mPFL",
       Patella, new Point3d(374 , 1400, 899),
       Femur, new Point3d(398, 1398, 840),
       new Point3d(403, 1398, 850), // Via point
       20000, 60.91, 5.0);
    
    //  Lateral patellofemoral ligament
    createLigament("lPFL",
       Patella, new Point3d(335, 1402, 891),
       Femur, new Point3d( 329, 1394, 837),
       new Point3d( 324, 1394, 847), // Via point
       20000, 54.92, 5.0);
    
}




private void addAchillesTendon() {
   // Create the fixed insertion point on the calcaneus
   Particle calcaneus = new Particle("Calcaneus", 0, 390.0, 965.0, 782.0);
   calcaneus.setName("Achilles_insertion");
   calcaneus.setDynamic(false);
   myMech.addParticle(calcaneus);
   RenderProps.setSphericalPoints(calcaneus, 2.0, Color.YELLOW);

   // Create the dynamic muscle attachment point
   Particle achillesPoint = new Particle("AchillesPoint", 0.01, 390.0, 1100.0, 800.0);
   achillesPoint.setName("AchillesPoint");
   achillesPoint.setDynamic(true); 
   myMech.addParticle(achillesPoint);
   this.achillesPoint = achillesPoint;
   
   RenderProps.setSphericalPoints(achillesPoint, 2.0, Color.YELLOW);
   double initialDistance = calcaneus.getPosition().distance(achillesPoint.getPosition());
   MultiPointSpring achillesTendon = new MultiPointSpring("AchillesTendon");
   achillesTendon.addPoint(calcaneus);
   achillesTendon.addPoint(achillesPoint);
   double stiffness = 5000000.0;     
   double referenceLength = initialDistance * 1.0;
   double damping = 0.05;             
   
   achillesTendon.setMaterial(
       new Blankevoort1991AxialLigament(stiffness, referenceLength, damping)
   );
   
   // Add the tendon to the mechanical model
   myMech.addMultiPointSpring(achillesTendon);
   
   // Set rendering properties for the tendon
   RenderProps.setCylindricalLines(achillesTendon, 1.5, new Color(0.9f, 0.7f, 0.2f)); 
   RenderProps.setVisible(achillesTendon, true);
   
   System.out.println("Achilles tendon added with reference length: " + referenceLength + " mm");
}
   
   private MultiPointSpring createLigament(String name,
      RigidBody bodyA, Point3d pointA,
      RigidBody bodyB, Point3d pointB,
      double stiffness, double refLength, double damping) {
// Create and attach markers
FrameMarker markerA = new FrameMarker();
FrameMarker markerB = new FrameMarker();
myMech.addFrameMarker(markerA, bodyA, pointA);
myMech.addFrameMarker(markerB, bodyB, pointB);

// Create the spring with the ligament material
MultiPointSpring ligament = new MultiPointSpring(name);
ligament.addPoint(markerA);
ligament.addPoint(markerB);
ligament.setMaterial(new Blankevoort1991AxialLigament(stiffness, refLength, damping));

// Set consistent render properties
RenderProps.setSphericalPoints(markerA, 1, Color.BLUE);
RenderProps.setSphericalPoints(markerB, 1, Color.BLUE);
RenderProps.setCylindricalLines(ligament, 0.3, Color.WHITE);

// Add to the mechanical model
myMech.addMultiPointSpring(ligament);
return ligament;
}

   private MultiPointSpring createLigament(String name,
      RigidBody bodyA, Point3d pointA,
      RigidBody bodyB, Point3d pointB,
      Point3d pointVia, // Via point parameters
      double stiffness, double refLength, double damping) {

FrameMarker markerA = new FrameMarker();
FrameMarker markerB = new FrameMarker();
Particle particleVia = new Particle(0.01,pointVia);

myMech.addFrameMarker(markerA, bodyA, pointA);
myMech.addFrameMarker(markerB, bodyB, pointB);
myMech.addParticle(particleVia);

MultiPointSpring ligament = new MultiPointSpring(name);
ligament.addPoint(markerA);
ligament.addPoint(particleVia); 
ligament.addPoint(markerB);
ligament.setSegmentWrappable (50) ; // wrappable segment
ligament.addWrappable(bodyA); 
ligament.addWrappable(bodyB);
ligament.setMaterial(new Blankevoort1991AxialLigament(stiffness, refLength, damping));
ligament. updateWrapSegments () ; // ‘‘ shrink wrap ’’ spring to the obstacles

RenderProps.setSphericalPoints(markerA, 1, Color.BLUE);
RenderProps.setSphericalPoints(markerB, 1, Color.BLUE);
RenderProps.setSphericalPoints(particleVia, 1, Color.GREEN); 
RenderProps.setCylindricalLines(ligament, 0.3, Color.WHITE);

myMech.addMultiPointSpring(ligament);
return ligament;
}


   private void addMuscle(RigidBody femur, RigidBody patella) {
      
      
      FrameMarker vastusLateralisOrigin = myMech.addFrameMarker(Femur, new Point3d(326.85349, 1587.8762, 845.98602) );
      vastusLateralisOrigin.setName(" Vastus_Lateralis_Origin");
      FrameMarker vastusLateralisInsertion = myMech.addFrameMarker(Patella, new Point3d(343.9282, 1410.4975, 894.20783));
      vastusLateralisInsertion.setName("Vastus_Lateralis_Insertion");
      RenderProps.setSphericalPoints(vastusLateralisOrigin, 2.5, Color.GREEN);
      RenderProps.setSphericalPoints(vastusLateralisInsertion, 2.5, Color.GREEN);

      MultiPointSpring vastusLateralis = new MultiPointSpring("Vastus_Lateralis");
      vastusLateralis.addPoint(vastusLateralisOrigin);  
      vastusLateralis.addPoint(vastusLateralisInsertion); 
      
      SimpleAxialMuscle muscleMatA = new SimpleAxialMuscle(2000, 0.5, 10000);
      vastusLateralis.setMaterial(muscleMatA);
      
      muscles.add(vastusLateralis);  // adds to the list
      myMech.addMultiPointSpring(vastusLateralis);
      
      RenderProps.setCylindricalLines (vastusLateralis, 0.8, Color.RED);
      RenderProps.setVisible(vastusLateralis, true);
      
      vastusLateralis.addWrappable(Femur);
      vastusLateralis.addWrappable(Patella);
      vastusLateralis.setSegmentWrappable(10);
            
      
      FrameMarker rectusFemorisOrigin = myMech.addFrameMarker(Femur, new Point3d(341.85349, 1587.8762, 845.98602) );
      rectusFemorisOrigin.setName(" Rectus_Femoris_Origin");
      FrameMarker rectusFemorisInsertion = myMech.addFrameMarker(Patella, new Point3d(355.54824, 1407.7581, 899.18102));
      rectusFemorisInsertion.setName("Rectus_Femoris_Insertion");
      RenderProps.setSphericalPoints(rectusFemorisOrigin, 2.5, Color.GREEN);
      RenderProps.setSphericalPoints(rectusFemorisInsertion, 2.5, Color.GREEN);

      MultiPointSpring rectusFemoris = new MultiPointSpring("Rectus_Femoris");
      rectusFemoris.addPoint(rectusFemorisOrigin);  
      rectusFemoris.addPoint(rectusFemorisInsertion); 
      
      SimpleAxialMuscle muscleMatB = new SimpleAxialMuscle(2000, 0.5, 100000);
      rectusFemoris.setMaterial(muscleMatB);
      
      muscles.add(rectusFemoris);  // adds to the list
      myMech.addMultiPointSpring(rectusFemoris);
      
      RenderProps.setCylindricalLines (rectusFemoris, 0.8, Color.RED);
      RenderProps.setVisible(rectusFemoris, true);
      
      rectusFemoris.addWrappable(Femur);
      rectusFemoris.addWrappable(Patella);
      rectusFemoris.setSegmentWrappable(10);
      
            
      FrameMarker vastusMedialisOrigin = myMech.addFrameMarker(Femur, new Point3d(356.85349, 1587.8762, 845.98602) );
      vastusMedialisOrigin.setName(" Vastus_Medialis_Origin");
      FrameMarker vastusMedialisInsertion = myMech.addFrameMarker(Patella, new Point3d(367.64791, 1410.5255, 895.69954));
      vastusMedialisInsertion.setName("Vastus_Medialis_Insertion");
      RenderProps.setSphericalPoints(vastusMedialisOrigin, 2.5, Color.GREEN);
      RenderProps.setSphericalPoints(vastusMedialisInsertion, 2.5, Color.GREEN);

      MultiPointSpring vastusMedialis = new MultiPointSpring("Vastus_Medialis");
      vastusMedialis.addPoint(vastusMedialisOrigin);  
      vastusMedialis.addPoint(vastusMedialisInsertion); 
      
      SimpleAxialMuscle muscleMatC = new SimpleAxialMuscle(2000, 0.5, 10000);
      vastusMedialis.setMaterial(muscleMatC);
      
      muscles.add(vastusMedialis);  // adds to the list
      myMech.addMultiPointSpring(vastusMedialis);
      
      RenderProps.setCylindricalLines (vastusMedialis, 0.8, Color.RED);
      RenderProps.setVisible(vastusMedialis, true);
      
      vastusMedialis.addWrappable(Femur);
      vastusMedialis.addWrappable(Patella);
      vastusMedialis.setSegmentWrappable(10);     
   }
   //Lateral:M. biceps femoris         Medial  M. semitendinosus & M. semimembranosus
   //324.46405 1343.5797 822.58936 TibiaFibula   334.84143 1664.3613 784.50519 Femur
   // 384.92139 1346.5948 822.41791 TibiaFiibula  337.82068 1694.3103 775.97101 Femur
   
   private void addSemitendinosusMuscle() {
      
      FrameMarker semitendinosusTibialInsertion = myMech.addFrameMarker( TibiaFibula, new Point3d(384.92139, 1346.5948, 822.41791) );
      semitendinosusTibialInsertion.setName("Semitendinosus_Tibial_Insertion");
      FrameMarker semitendinosusFemoralOrigin = myMech.addFrameMarker(Femur, new Point3d(375.82068, 1694.3103, 765.97101) );
      semitendinosusFemoralOrigin.setName("Semitendinosus_Femoral_Origin");
      RenderProps.setSphericalPoints(semitendinosusTibialInsertion, 2.5, Color.GREEN);
      RenderProps.setSphericalPoints(semitendinosusFemoralOrigin, 2.5, Color.GREEN);
      MultiPointSpring semitendinosus = new MultiPointSpring("Semitendinosus");
      semitendinosus.addPoint(semitendinosusFemoralOrigin);   
      semitendinosus.addPoint(semitendinosusTibialInsertion); 

      SimpleAxialMuscle muscleMat = new SimpleAxialMuscle(2000, 0.5, 20000);
      semitendinosus.setMaterial(muscleMat);
      
      muscles.add(semitendinosus);  // adds to the list
      
      myMech.addMultiPointSpring(semitendinosus);
   
      RenderProps.setCylindricalLines (semitendinosus, 0.8, Color.RED);
      RenderProps.setVisible(semitendinosus, true);
      
      semitendinosus.addWrappable(Femur);
      semitendinosus.addWrappable(TibiaFibula);
      semitendinosus.setSegmentWrappable(10);
  }


   
   private MultiPointSpring addBicepsFemorisMuscle() {
      
      FrameMarker bicepsTibialInsertion = myMech.addFrameMarker(TibiaFibula, new Point3d(324.46405, 1343.5797, 822.58936) );
      bicepsTibialInsertion.setName("Biceps_Tibial_Insertion");
      FrameMarker bicepsFemoralOrigin = myMech.addFrameMarker(Femur,  new Point3d(360.84143, 1694.3613, 774.50519));
      bicepsFemoralOrigin.setName("Biceps_Femoral_Origin");
      RenderProps.setSphericalPoints(bicepsTibialInsertion, 2.5, Color.GREEN);
      RenderProps.setSphericalPoints(bicepsFemoralOrigin, 2.5, Color.GREEN);

      MultiPointSpring bicepsFemoris = new MultiPointSpring("Biceps_Femoris");
      bicepsFemoris.addPoint(bicepsFemoralOrigin);  
      bicepsFemoris.addPoint(bicepsTibialInsertion); 
      
      SimpleAxialMuscle muscleMat = new SimpleAxialMuscle(2000, 0.5, 20000);
      bicepsFemoris.setMaterial(muscleMat);
      
      muscles.add(bicepsFemoris);  // adds to the list

      myMech.addMultiPointSpring(bicepsFemoris);
      
      RenderProps.setCylindricalLines (bicepsFemoris, 0.8, Color.RED);
      RenderProps.setVisible(bicepsFemoris, true);
      
       bicepsFemoris.addWrappable(Femur);
       bicepsFemoris.addWrappable(TibiaFibula);
       bicepsFemoris.setSegmentWrappable(10);
       
       return bicepsFemoris;
     
      
     
  }
   
   private void addSoleusMuscle() {
      FrameMarker soleusTibialInsertion = myMech.addFrameMarker(TibiaFibula, new Point3d(364.1702, 1264.9733, 827.86859) );
      soleusTibialInsertion.setName("Soleus_Tibial_Insertion");
      RenderProps.setSphericalPoints(soleusTibialInsertion, 2.5, Color.GREEN);
      
      MultiPointSpring soleus = new MultiPointSpring("Soleus");
     
      soleus.addPoint(soleusTibialInsertion);
      soleus.addPoint(achillesPoint);        
      soleus.setMaterial (new SimpleAxialMuscle (2000, 0.5, 20000));
      
      muscles.add(soleus);  // adds to the list

      myMech.addMultiPointSpring (soleus);
      RenderProps.setCylindricalLines (soleus, 0.8, Color.RED);
      RenderProps.setVisible(soleus, true);
  }
   
   
private void addGastrocnemiusMuscles() {
      
      FrameMarker gastrocLatOrigin = myMech.addFrameMarker(Femur,  new Point3d(343.21899, 1395.6849, 825.1441) );
      gastrocLatOrigin.setName("Gastroc_Lat_Origin");
      RenderProps.setSphericalPoints(gastrocLatOrigin, 2.5, Color.GREEN);
      
      FrameMarker gastrocMedOrigin = myMech.addFrameMarker(Femur,  new Point3d(382.71698, 1395.064, 829.1026) );
      gastrocMedOrigin.setName("Gastroc_Med_Origin");
      RenderProps.setSphericalPoints(gastrocMedOrigin, 2.5, Color.GREEN);

      MultiPointSpring gastrocLat = new MultiPointSpring("Gastrocnemius_Lateral");
      
      FrameMarker gastrocLatVia1 = myMech.addFrameMarker(Femur, new Point3d(343.21899, 1398.6849, 819.1441));
      gastrocLatVia1.setName("Gastroc_Lat_Via1");
      RenderProps.setSphericalPoints(gastrocLatVia1, 1.5, Color.CYAN); 
      
      FrameMarker gastrocLatVia2 = myMech.addFrameMarker(Femur, new Point3d(343.21899, 1395.6849, 814.1441));
      gastrocLatVia2.setName("Gastroc_Lat_Via2");
      RenderProps.setSphericalPoints(gastrocLatVia2, 1.5, Color.CYAN);
      
      FrameMarker gastrocMedVia1 = myMech.addFrameMarker(Femur, new Point3d(382.71698, 1398.064, 822.1026));
      gastrocMedVia1.setName("Gastroc_Med_Via1");
      RenderProps.setSphericalPoints(gastrocMedVia1, 1.5, Color.CYAN);
      
      FrameMarker gastrocMedVia2 = myMech.addFrameMarker(Femur, new Point3d(382.71698, 1395.064, 816.1026));
      gastrocLatVia2.setName("Gastroc_Med_Via2");
      RenderProps.setSphericalPoints(gastrocMedVia2, 1.5, Color.CYAN);
      
      gastrocLat.addPoint(gastrocLatOrigin);
      gastrocLat.addPoint(gastrocLatVia1);
      gastrocLat.addPoint(gastrocLatVia2);
      gastrocLat.addPoint(achillesPoint);  
      
      SimpleAxialMuscle latMuscleMat = new SimpleAxialMuscle(2000, 0.5, 20000);
      gastrocLat.setMaterial(latMuscleMat);
      
      muscles.add(gastrocLat);  // adds to the list
      myMech.addMultiPointSpring(gastrocLat);
      RenderProps.setCylindricalLines (gastrocLat, 0.8, Color.RED);
      
          
      MultiPointSpring gastrocMed = new MultiPointSpring("Gastrocnemius_Medial");
      gastrocMed.addPoint(gastrocMedOrigin);
      gastrocMed.addPoint(gastrocMedVia1);
      gastrocMed.addPoint(gastrocMedVia2);
      gastrocMed.addPoint(achillesPoint);  
            
      SimpleAxialMuscle medMuscleMat = new SimpleAxialMuscle(2000, 0.5, 20000);
      medMuscleMat.setDamping(0.6);
      gastrocMed.setMaterial(medMuscleMat);
      
      muscles.add(gastrocMed);  // adds to the list
      myMech.addMultiPointSpring(gastrocMed);
      RenderProps.setCylindricalLines (gastrocMed, 0.8, Color.RED);
      
     
      gastrocLat.addWrappable(Femur);
      gastrocLat.addWrappable(TibiaFibula);
      gastrocLat.setSegmentWrappable(10); 
      gastrocMed.addWrappable(Femur);
      gastrocMed.addWrappable(TibiaFibula);
      gastrocMed.setSegmentWrappable(10);
   }

   private void addTibialisAnteriorMuscle() {
      
      FrameMarker tibAntOrigin = myMech.addFrameMarker(TibiaFibula,  new Point3d(345.33441, 1312.1631, 841.00348) );
      tibAntOrigin.setName("Tibialis_Anterior_Origin");
      RenderProps.setSphericalPoints(tibAntOrigin, 2.5, Color.GREEN); 
          
      MultiPointSpring tibialisAnterior = new MultiPointSpring("Tibialis_Anterior");
      tibialisAnterior.addPoint(tibAntOrigin);
      tibialisAnterior.addPoint(cuneiforme);
      
      SimpleAxialMuscle muscleMat = new SimpleAxialMuscle(2000, 0.5, 20000);
      tibialisAnterior.setMaterial(muscleMat);
      
      muscles.add(tibialisAnterior);  // adds to the list

      myMech.addMultiPointSpring(tibialisAnterior);
      RenderProps.setCylindricalLines (tibialisAnterior, 0.8, Color.RED);
        

  }




   
 // #############################################################################################################################
   

   private JointBase createHingeJointToGround(ConnectableBody body) {
      Vector3d origin = new Vector3d(365, 967, 800);
      RigidTransform3d TDW = new RigidTransform3d(origin.x, origin.y, origin.z);
      TDW.setRpyDeg(0, 90, 0); 
      HingeJoint joint = new HingeJoint(body, TDW);
      myMech.addBodyConnector(joint);
      //setJointCompliance(joint, 1e-7, 1e5);
      setJointCompliance(joint,1e-3, 1e4, 1e-12, 1e9);
      setJointRenderProps(joint);
      JointControl(joint);
      return joint;
  }
   
   private void setJointCompliance(JointBase joint, double rotCompliance, double rotDamping, double lockCompliance, double lockDamping) {
      VectorNd comp = new VectorNd(joint.numConstraints());
      VectorNd damp = new VectorNd(joint.numConstraints());

      int rotationConstraintIndex = 5; // it is the last one!

      for (int i = 0; i < joint.numConstraints(); i++) {
          if (i == rotationConstraintIndex) {
              // apply softer compliance to the rotational constraint
              comp.set(i, rotCompliance);
              damp.set(i, rotDamping);
          } else {
              // extremely stiff compliance to LOCK the other constraints
              comp.set(i, lockCompliance);   
              damp.set(i, lockDamping);      
          }
      }
      joint.setCompliance(comp);
      joint.setDamping(damp);
  }
   


   private void setJointRenderProps (JointBase joint) {
      joint.setShaftLength (70);
      joint.setShaftRadius (1);
      joint.setAxisLength (50);
      joint.setDrawFrameC (AxisDrawStyle.ARROW);
      joint.setDrawFrameD (AxisDrawStyle.ARROW);
   }

   private void JointControl (JointBase joint) {
      ControlPanel panel = new ControlPanel ();
      panel.addWidget (joint, "theta");
      panel.addWidget (joint, "compliance");
      panel.addWidget (joint, "damping");
      addControlPanel (panel);
   }


   
   private void setCollisionBehavior (
      Collidable body1, Collidable body2, double mu, double compliance,
      double damping) {
      CollisionBehavior behavior = new CollisionBehavior (true, mu);
      behavior.setCompliance (compliance);
      behavior.setDamping (damping);
      behavior.setPenetrationTol (0.0001);                                        // Achtung! vieleicht zu viel
 //     behavior.setMethod ( CollisionBehavior . Method.VERTEX_PENETRATION );       // Achtung! VERTEX_EDGE_PENETRATION?
      behavior.setColliderType(CollisionManager.ColliderType.TRI_INTERSECTION);         // testing
      behavior.setBilateralVertexContact(true);                                    //s usually improves Performance ?
      behavior.setMethod(CollisionBehavior . Method.VERTEX_PENETRATION_BILATERAL);  // better?
      myMech.setCollisionBehavior (body1, body2, behavior);
   }

   private void setCollisionManager () {
      CollisionManager cm = myMech.getCollisionManager ();
      RenderProps.setVisible (cm, false);//
      cm.setReduceConstraints (true);           
      cm.setDrawContactForces (false);
      cm.setDrawFrictionForces (false);
      cm.setContactForceLenScale (0.1);
      cm.setSmoothVertexContacts (true);
      RenderProps.setSolidArrowLines (cm, 0.2, Color.RED);
      //cm.setDrawIntersectionPoints (true);
      //RenderProps.setSphericalPoints (cm, 0.5, Color.GREEN);
   }
   
   private void printFemModelInfo(FemModel3d fem, String label) {
      System.out.println("================================");
      System.out.println("--- " + label + " ---");
      System.out.println("Nodes: " + fem.numNodes());
      System.out.println("Elements: " + fem.numElements());
      System.out.println("================================");
  }
   
   
/*   
   private void setCollisionBehavior (
      Collidable body1, Collidable body2, double mu, double compliance,
      double damping) {
      CollisionBehavior behavior = new CollisionBehavior (true, mu);
      behavior.setCompliance (compliance);
      behavior.setDamping (damping);
      myMech.setCollisionBehavior (body1, body2, behavior);
   }

   private void setCollisionManager () {
      CollisionManager cm = myMech.getCollisionManager ();
      RenderProps.setVisible (cm, true);
      cm.setDrawContactForces (false);
      cm.setDrawFrictionForces (false);
      cm.setContactForceLenScale (0.1);
      RenderProps.setSolidArrowLines (cm, 0.2, Color.RED);
      cm.setDrawIntersectionPoints (true);
      RenderProps.setSphericalPoints (cm, 0.5, Color.GREEN);
   }

   private class ContactMonitor extends MonitorBase {
      public void apply (double t0, double t1) {
         // get the contacts from the collision response and print their
         // positions and forces.
         List<ContactData> cdata = myResp.getContactData ();
         if (cdata.size () > 0) {
            System.out
               .println ("num contacts: " + cdata.size () + ", time=" + t0);
            double contactSum = 0;
            for (ContactData cd : cdata) {
               System.out
                  .print (" pos:   " + cd.getPosition0 ().toString ("%8.3f"));
               System.out
                  .println (
                     ", force: " + cd.getContactForce ().toString ("%8.1f"));
               contactSum = contactSum + cd.getContactForceScalar ();
            }
         }
      }
   }
*/
   
// Add Von Mises writer declaration

public PrintWriter writerStress;
public PrintWriter writerStrain;
public PrintWriter writerMises;  

// Your existing declarations
private Set<Integer> targetNodeNumbers = new HashSet<>();
private List<FemNode3d> targetNodes = new ArrayList<>();

private void initialWriter() throws IOException {
   try {
       writerStress = new PrintWriter(new FileWriter(myPath + "output/Stress.txt", true));
       writerStrain = new PrintWriter(new FileWriter(myPath + "output/Strain.txt", true));
       writerMises = new PrintWriter(new FileWriter(myPath + "output/VonMisesStress.txt", true));
       
       // Headers
       writerStress.println("=== Stress Data for Randomly Selected Nodes ===");
       writerStress.println("Target Nodes: " + targetNodeNumbers);
       writerStress.println("Format: Time[s], NodeID, StressTensor");
       
       writerStrain.println("=== Strain Data for Randomly Selected Nodes ===");
       writerStrain.println("Target Nodes: " + targetNodeNumbers);
       writerStrain.println("Format: Time[s], NodeID, StrainTensor");
       
       writerMises.println("Time[s]\tNodeID\tVonMisesStress[MPa]");
       
   }
   catch (IOException e) {
       e.printStackTrace();
   }
}

private void addProbe() throws IOException {

    initializeTargetNodes();
    
    NumericMonitorProbe StressProbe = new NumericMonitorProbe(
        targetNodes.size(), myPath + "output/Stress.dat", 0, 20, -1);
    StressProbe.setName("Stress");
    StressProbe.setDataFunction(new FEMStressFunction());
    addOutputProbe(StressProbe);

    NumericMonitorProbe StrainProbe = new NumericMonitorProbe(
        targetNodes.size(), myPath + "output/Strain.dat", 0, 20, -1);
    StrainProbe.setName("Strain");
    StrainProbe.setDataFunction(new FEMStrainFunction());
    addOutputProbe(StrainProbe);
    
    NumericMonitorProbe VonMisesProbe = new NumericMonitorProbe(
        targetNodes.size(), myPath + "output/VonMisesStress.dat", 0, 20, -1);
    VonMisesProbe.setName("VonMisesStress");
    VonMisesProbe.setDataFunction(new FEMVonMisesFunction());
    addOutputProbe(VonMisesProbe);
}

private void initializeTargetNodes() {
    targetNodeNumbers.clear();
    targetNodes.clear();
    
    List<FemNode3d> allNodes = new ArrayList<>();
    for (FemNode3d node : meshMeniscus.getNodes()) {
        allNodes.add(node);
    }
    
    Random random = new Random();
    int sampleSize = Math.min(30, allNodes.size());
    
    Collections.shuffle(allNodes, random);
    
    for (int i = 0; i < sampleSize; i++) {
        FemNode3d selectedNode = allNodes.get(i);
        targetNodeNumbers.add(selectedNode.getNumber());
        targetNodes.add(selectedNode);
    }
    
    System.out.println("Randomly selected " + targetNodes.size() + " nodes:");
    for (FemNode3d node : targetNodes) {
        System.out.println("  Node: " + node.getNumber());
    }
    
    for (FemNode3d node : targetNodes) {
        System.out.println("  Node: " + node.getNumber());
        RenderProps.setPointStyle(node, PointStyle.SPHERE);
        RenderProps.setPointRadius(node, 0.6); 
        RenderProps.setPointColor(node, Color.ORANGE); 

    }
}


public class FEMStressFunction implements DataFunction, Clonable {
    public void eval(VectorNd vec, double t, double trel) {
        writerStress.println("Time: " + t);
        
        int idx = 0;
        for (FemNode3d n : targetNodes) {
            vec.set(idx++, n.getStress().m00);
            
            SymmetricMatrix3d stress = n.getStress();
            writerStress.println("Node " + n.getNumber() + ":");
            writerStress.printf("%.6f   %.6f   %.6f%n", stress.m00, stress.m01, stress.m02);
            writerStress.printf("%.6f   %.6f   %.6f%n", stress.m01, stress.m11, stress.m12);
            writerStress.printf("%.6f   %.6f   %.6f%n", stress.m02, stress.m12, stress.m22);
        }
        writerStress.println();
        writerStress.flush();
    }

    public Object clone() throws CloneNotSupportedException {
        return super.clone();
    }
}


public class FEMStrainFunction implements DataFunction, Clonable {
    public void eval(VectorNd vec, double t, double trel) {
        writerStrain.println("Time: " + t);
        
        int idx = 0;
        for (FemNode3d n : targetNodes) {
            vec.set(idx++, n.getStrain().m00);
            
            SymmetricMatrix3d strain = n.getStrain();
            writerStrain.println("Node " + n.getNumber() + ":");
            writerStrain.printf("%.6f   %.6f   %.6f%n", strain.m00, strain.m01, strain.m02);
            writerStrain.printf("%.6f   %.6f   %.6f%n", strain.m01, strain.m11, strain.m12);
            writerStrain.printf("%.6f   %.6f   %.6f%n", strain.m02, strain.m12, strain.m22);
        }
        writerStrain.println();
        writerStrain.flush();
    }

    public Object clone() throws CloneNotSupportedException {
        return super.clone();
    }
}


public class FEMVonMisesFunction implements DataFunction, Clonable {
    public void eval(VectorNd vec, double t, double trel) {
        for (int i = 0; i < targetNodes.size(); i++) {
            FemNode3d n = targetNodes.get(i);
            double vonMisesStress = n.getVonMisesStress();
            vec.set(i, vonMisesStress);
            writerMises.printf("%.3f\t%d\t%.6f%n", t, n.getNumber(), vonMisesStress);
        }
        writerMises.flush();
    }

    public Object clone() throws CloneNotSupportedException {
        return super.clone();
    }
    
}

}