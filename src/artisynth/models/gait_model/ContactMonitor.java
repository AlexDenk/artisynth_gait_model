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
 * per joint with the contact parameters written to a
 * {@code*_message_file.txt} in the current working directory.
 * <p>
 * 
 * @author Alexander Denk Copyright (c) 2024
 * <p>
 * University of Duisburg-Essen
 * <p>
 * Chair of Mechanics and Robotics
 * <p>
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
   // ----------------------------Nested Classes ------------------------------

   // -----------------------------Constructors--------------------------------
   /**
    * Generates a contact monitor object, that monitors the collision responses
    * listed in {@code resp} and writes those contact events to a file
    * corresponding to the current working directory.
    * <p>
    * Example: For a model residing in some folder {@code C:\...\sim1}, the
    * ContactMonitor will write the contact events to
    * {@code C:\...\sim1\Output\sim1_message.txt}.
    * 
    * @param resp
    * collision response list
    * @param name
    * working directory identifier
    * @throws IOException
    */
   public ContactMonitor (CollisionResponseList resp, String name)
   throws IOException {
      super ();
      initializeWriter (name);
      // Add each item in the CollisionResponseList to a separate list, since
      // there seem to be iteration problems with the CollisionResponseList
      // class
      resp.forEach (r -> {
         collResp.add (r);
      });
   }

   // ----------------------------Instance Methods-----------------------------
   @Override
   public void initialize (double t0) {
      super.initialize (t0);
      // Close writer, if it was priorly active
      if (isActive)
         writer.close ();
      isActive = true;
   }

   public void apply (double t0, double t1) {
      writeContactToFile (t0);
   }

   /**
    * Writes the generated messages to file.
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
         contactEvents
            .append ("COLLISION INTERFACE: " + cr.getName () + "\n");
         if (cr.inContact ()) {
            List<ContactData> cdata = cr.getContactData ();
            if (cdata.size () > 0) {
               contactEvents
                  .append (
                     "FOUND " + cdata.size () + " CONTACT EVENTS." + "\n");
               cdata.forEach (cd -> {
                  String row =
                     String
                        .format (
                           "%-20s%-6s", "POSITION",
                           cd.getPosition0 ().toString ("%.3f"));
                  contactEvents.append (row + "\n");
                  row =
                     String
                        .format (
                           "%-20s%-6s", "CONTACT FORCE (N)",
                           cd.getContactForce ().toString ("%.3f"));
                  contactEvents.append (row + "\n");
                  row =
                     String
                        .format (
                           "%-20s%-6s", "FRICTION FORCE (N)",
                           cd.getFrictionForce ().toString ("%.3f"));
                  contactEvents.append (row + "\n\n");
               });
            }
         }
         else {
            contactEvents.append ("NO CONTACT DETECTED." + "\n\n");
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