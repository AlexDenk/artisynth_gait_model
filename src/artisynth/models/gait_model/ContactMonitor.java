package artisynth.models.gait_model;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;

import artisynth.core.mechmodels.CollisionResponse;
import artisynth.core.mechmodels.CollisionResponseList;
import artisynth.core.mechmodels.ContactData;
import artisynth.core.modelbase.MonitorBase;
import artisynth.core.util.ArtisynthPath;
import artisynth.models.gait_model.Tests.GaitModel;

/**
 * The contact monitor acts as a monitor (see artisynth manual for further
 * details), to supervise contact events during simulation. Contact is monitored
 * per joint with the contact parameters written to a *_message_file.txt in the
 * current working directory.
 * 
 * @author Alexander Denk Copyright (c) 2023, by the Author: Alexander Denk
 * (UDE) University of Duisburg-Essen Chair of Mechanics and Robotics
 * alexander.denk@uni-due.de
 */

public class ContactMonitor extends MonitorBase {
   // ----------------------------Instance Fields------------------------------
   // Name of the file, where the contact history is written to
   String msgName = null;
   // Path to the file, where the contact history is written to
   String msgPath = null;
   // Writer object, that writes data to file
   PrintWriter writer;
   // Writer variable, that tracks, whether the writer was active before
   boolean isActive;
   // A list of all available collision pairs
   List<CollisionResponse> collResp = new ArrayList<CollisionResponse> ();
   // A list of all collision pairs, that are actively monitored
   List<CollisionResponse> activeResp = new ArrayList<CollisionResponse> ();
   // Checks if all available collision pairs are actively monitored
   boolean useFullReport;
   // ----------------------------Nested Classes ------------------------------

   // -----------------------------Constructors--------------------------------
   public ContactMonitor (CollisionResponseList resp, String name)
   throws IOException {
      super ();
      initializeWriter (name);
      // Add each item in the CollisionResponseList to a separate list, since
      // there are iteration problems with the CollisionResponseList class
      resp.forEach (r -> {
         collResp.add (r);
      });

   }

   // ----------------------------Instance Methods-----------------------------
   @Override
   public void initialize (double t0) {
      super.initialize (t0);
      // Close writer, if it was priorly active
      if (isActive) {
         writer.close ();
      }
      isActive = true;
      // Check for the report mode
      if (!useFullReport) {
         // Search for all responses regarding toes and calcanei
         collResp.forEach (cr -> {
            if (cr.getName ().contains ("ground")
            && cr.getName ().contains ("toes")) {
               activeResp.add (cr);
            }
            else if (cr.getName ().contains ("ground")
            && cr.getName ().contains ("calcn")) {
               activeResp.add (cr);
            }
         });
      }
      else {
         activeResp = collResp;
      }
   }

   public void apply (double t0, double t1) {
      // TODO: get the actual penetration depth of each contact event.
      writeContactToFile (t0);
   }

   /**
    * Queries whether full report mode is used by this {@link ContactMonitor}
    * 
    * @return
    */
   public boolean useFullReport () {
      return useFullReport;
   }

   /**
    * Enables full report if set to true. If full report is desired, then the
    * contact info of all collision behaviors is written to file. If unused,
    * only the ground contact will be written to file.
    * 
    * @param b
    */
   public void setUseFullReport (boolean b) {
      useFullReport = b;
   }

   /**
    * Writes the generated messages to file.
    * 
    * @param t0
    */
   private void writeContactToFile (double t0) {
      StringBuilder contactEvents = new StringBuilder ();
      contactEvents
         .append (
            System.lineSeparator () + "DETECT CONTACT EVENTS"
            + System.lineSeparator ());
      activeResp.forEach (cr -> {
         contactEvents
            .append (
               System.lineSeparator () + "COLLISION INTERFACE: " + cr.getName ()
               + System.lineSeparator ());
         if (cr.inContact ()) {
            List<ContactData> cdata = cr.getContactData ();
            if (cdata.size () > 0) {
               contactEvents
                  .append (
                     "FOUND " + cdata.size () + " CONTACT EVENTS."
                     + System.lineSeparator ());
               cdata.forEach (cd -> {
                  String row =
                     String
                        .format (
                           "%-20s%-6s", "POSITION",
                           cd.getPosition0 ().toString ("%.3f"));
                  contactEvents.append (row + System.lineSeparator ());
                  row =
                     String
                        .format (
                           "%-20s%-6s", "CONTACT FORCE (N)",
                           cd.getContactForce ().toString ("%.3f"));
                  contactEvents.append (row + System.lineSeparator ());
                  row =
                     String
                        .format (
                           "%-20s%-6s", "FRICTION FORCE (N)",
                           cd.getFrictionForce ().toString ("%.3f"));
                  contactEvents.append (row + System.lineSeparator ());
               });
            }
         }
         else {
            contactEvents
               .append ("NO CONTACT DETECTED." + System.lineSeparator ());
         }
      });
      writer.print (contactEvents.toString ());
      writer.flush ();
      contactEvents.delete (0, contactEvents.length ());
   }

   /**
    * Initializes PrintWriter from constructor.
    * 
    * @param name
    * Name specifier for the current working directory
    * @throws IOException
    */
   private void initializeWriter (String name) throws IOException {
      this.msgName = name + "/Output/" + name + "_message_file.txt";
      this.msgPath =
         ArtisynthPath
            .getSrcRelativePath (GaitModel.class, msgName).toString ();
      writer = new PrintWriter (new FileWriter (msgPath, false));
      this.isActive = false;
   }
}