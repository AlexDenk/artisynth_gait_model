package artisynth.models.gait_model.helper_classes;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import artisynth.core.femmodels.FemMeshComp;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.PointFem3dAttachment;
import artisynth.core.mechmodels.Collidable;
import artisynth.core.mechmodels.CollisionResponse;
import artisynth.core.mechmodels.CollisionResponseList;
import artisynth.core.mechmodels.ContactData;
import artisynth.core.mechmodels.PointAttachment;
import artisynth.core.modelbase.ContactPoint;
import artisynth.core.modelbase.MonitorBase;
import maspack.geometry.Vertex3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;

/**
 * The {@code ContactMonitor} subclasses {@link MonitorBase} to collect and
 * store contact events during simulation. Contact is monitored per collision
 * response with the contact parameters written to file.
 * <p>
 * Besides the contact position and the contact forces, each contact
 * is resolved to a node ID of the underlying body. For finite element
 * collidables ({@link FemMeshComp}, or a {@link FemModel3d} via its surface
 * mesh) the dominant (highest-weight) FEM node number is reported; for rigid
 * bodies the mesh vertex index is reported instead, prefixed with {@code v} to
 * distinguish it from a FEM node number. Where the collision provides
 * per-vertex contact forces (vertex penetration on a deformable body), an
 * aggregated per-node force / pressure table is appended as well.
 * <p>
 *
 * @author Alexander Denk Copyright (c) 2026
 * <p>
 * University of Duisburg-Essen
 * <p>
 * Chair of Mechanics and Robotics
 * <p>
 * alexander.denk@uni-due.de
 */

public class ContactMonitor extends MonitorBase {
   // ----------------------------Instance Fields------------------------------
   // Path to the file, where the contact history is written to
   String msgPath = null;
   // Writer object, that writes data to file
   PrintWriter writer;
   // Writer variable, that tracks, whether the writer was active before
   boolean isActive;
   // A list of all available collision pairs
   List<CollisionResponse> collResp = new ArrayList<CollisionResponse> ();
   // ----------------------------Nested Classes ------------------------------

   // -----------------------------Constructors--------------------------------
   /**
    * Generates a contact monitor object, that monitors the collision responses
    * listed in {@code resp} and writes those contact events to a file located
    * at {@code filepath}.
    *
    * @param resp
    * collision response list
    * @param filepath
    * path to the file
    * @throws IOException
    */
   public ContactMonitor (CollisionResponseList resp, String filepath)
   throws IOException {
      super ();
      initializeWriter (filepath);
      // Add each item in the CollisionResponseList to a separate list, since
      // there seem to be iteration problems with the CollisionResponseList
      // class
      resp.forEach (r -> {
         collResp.add (r);
      });
   }

   public ContactMonitor () {

   }

   // ----------------------------Instance Methods-----------------------------
   @Override
   public void initialize (double t0) {
      super.initialize (t0);
      // Close writer, if it was priorly active
      if (isActive)
         writer.close ();
      isActive = true;
      writeHeaderToFile ();
   }

   public void apply (double t0, double t1) {
      writeContactToFile (t0);
   }

   private String collectContactEvents (CollisionResponse cr) {
      StringBuilder contacts = new StringBuilder ();
      contacts.append ("COLLISION INTERFACE: " + cr.getName () + "\n");
      if (cr.inContact ()) {
         List<ContactData> cdata = cr.getContactData ();
         contacts.append ("FOUND " + cdata.size () + " CONTACT EVENTS." + "\n");
         // Per-constraint contact table. NodeID and Position both refer to the
         // first contact point of the constraint.
         Collidable col0 = cr.getCollidable (0);
         String c0 = appendContactData (cr, col0, 0);
         contacts.append (c0);

         Collidable col1 = cr.getCollidable (1);
         String c1 = appendContactData (cr, col1, 1);
         contacts.append (c1);
      }
      else {
         contacts.append ("NO CONTACT DETECTED." + "\n\n");
      }
      return contacts.toString ();
   }

   private String appendContactData (
      CollisionResponse cr, Collidable col, int cidx) {
      StringBuilder contact = new StringBuilder ();
      String name =
         (col != null && col.getName () != null) ? col.getName ()
            : "collidable " + cidx;
      contact.append ("PER-NODE CONTACT DATA (" + name + ")\n");
      String format = "%-7s%-20s%-30s%-20s%n";
      String dataHeader =
         String
            .format (
               format, "NodeID", "Position", "Contact force (N)",
               "Pressure (Pa)");
      contact.append (dataHeader);

      Map<Vertex3d,Vector3d> forces = cr.getContactForces (cidx);
      if (forces.isEmpty ()) {
         return null;
      }
      Map<Vertex3d,Double> pressures = cr.getContactPressures (cidx);
      for (Map.Entry<Vertex3d,Vector3d> entry : forces.entrySet ()) {
         Vertex3d vtx = entry.getKey ();
         Double p = pressures.get (vtx);
         contact
            .append (
               String
                  .format (
                     format, nodeId (col, vtx),
                     vtx.getWorldPoint ().toString ("%.3f"),
                     entry.getValue ().toString ("%.3f"),
                     p != null ? String.format ("%.3f", p) : "-"));
      }
      contact.append ("\n");
      return contact.toString ();
   }

   /**
    * Returns the node ID token for mesh vertex {@code vtx} on collidable
    * {@code col}: the dominant FEM node number for a finite element collidable,
    * or {@code v<index>} (the mesh vertex index) for a rigid body or any vertex
    * that cannot be resolved to a single FEM node. Returns {@code "-"} when
    * there is no associated vertex.
    */
   private String nodeId (Collidable col, Vertex3d vtx) {
      if (vtx == null) {
         return "-";
      }
      FemMeshComp fmc = null;
      if (col instanceof FemMeshComp) {
         fmc = (FemMeshComp)col;
      }
      if (col instanceof FemModel3d) {
         fmc = ((FemModel3d)col).getSurfaceMeshComp ();
      }
      if (fmc != null) {
         FemNode3d node = dominantNode (fmc, vtx);
         if (node != null) {
            return String.valueOf (node.getNumber ());
         }
      }
      return "v" + vtx.getIndex ();
   }

   /**
    * Returns the FEM node that dominates mesh vertex {@code vtx} of
    * {@code fmc}. For a vertex attached to a single node that node is returned;
    * for an embedded / multi-node vertex the highest-weight master node is
    * returned. Returns {@code null} if no FEM node can be resolved.
    */
   private FemNode3d dominantNode (FemMeshComp fmc, Vertex3d vtx) {
      FemNode3d node = fmc.getNodeForVertex (vtx);
      if (node != null) {
         return node;
      }
      // Embedded / multi-node vertex: pick the highest-weight master node.
      if (vtx.getIndex () < fmc.numVertexAttachments ()) {
         PointAttachment pa = fmc.getVertexAttachment (vtx);
         if (pa instanceof PointFem3dAttachment) {
            PointFem3dAttachment pfa = (PointFem3dAttachment)pa;
            FemNode[] nodes = pfa.getNodes ();
            VectorNd wgts = pfa.getCoordinates ();
            int best = -1;
            double bestWgt = Double.NEGATIVE_INFINITY;
            for (int i = 0; i < nodes.length && i < wgts.size (); i++) {
               if (wgts.get (i) > bestWgt) {
                  bestWgt = wgts.get (i);
                  best = i;
               }
            }
            if (best >= 0 && nodes[best] instanceof FemNode3d) {
               return (FemNode3d)nodes[best];
            }
         }
      }
      return null;
   }

   /**
    * Writes contact data to file.
    *
    * @param t0
    * current time
    */
   private void writeContactToFile (double t0) {
      StringBuilder contactEvents = new StringBuilder ();
      contactEvents
         .append (
            "----------------------------- TIME " + t0
            + "-----------------------------\n\n")
         .append ("DETECT CONTACT EVENTS" + "\n\n");
      collResp.forEach (cr -> {
         contactEvents.append (collectContactEvents (cr));
      });
      writer.print (contactEvents.toString ());
      writer.flush ();
      contactEvents.delete (0, contactEvents.length ());
   }

   private void writeHeaderToFile () {
      StringBuilder header = new StringBuilder ();
      header
         .append (
            "%%-------------------- CONTACT HISTORY FILE ------------------%%\n")
         .append (
            "%% Author: Alexander Denk, Copyright (c) 2026                 %%\n")
         .append (
            "%% (UDE) University of Duisburg-Essen                         %%\n")
         .append (
            "%% Chair of Mechanics and Robotics                            %%\n")
         .append (
            "%% alexander.denk@uni-due.de                                  %%\n")
         .append (
            "%% NodeID: integer = FEM node number, v<idx> = mesh vertex    %%\n")
         .append (
            "%%------------------------------------------------------------%%\n");
      writer.print (header.toString ());
      writer.flush ();
   }

   /**
    * Initializes PrintWriter from constructor.
    *
    * @param name
    * Name specifier for the current working directory
    * @throws IOException
    */
   private void initializeWriter (String filepath) throws IOException {
      this.msgPath = filepath;
      writer = new PrintWriter (new FileWriter (msgPath, false));
      this.isActive = false;
   }
}
