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

/**
 * The contact monitor acts as a monitor (see artisynth manual for further
 * details), to supervise contact events during simulation. Contact is monitored
 * per collision response with the contact parameters written to file.
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

   private String collectContactEvents (CollisionResponse cr) {
      StringBuilder contacts = new StringBuilder ();
      contacts.append ("COLLISION INTERFACE: " + cr.getName () + "\n");
      if (cr.inContact ()) {
         List<ContactData> cdata = cr.getContactData ();
         contacts.append ("FOUND " + cdata.size () + " CONTACT EVENTS." + "\n");
         if (cdata.size () == 0)
            return contacts.toString ();
         cdata.forEach (cd -> {
            String row = String.format ("%-20s%-6s", "POSITION",
                     cd.getPosition0 ().toString ("%.3f"));
            contacts.append (row + "\n");
            row = String.format ("%-20s%-6s", "CONTACT FORCE (N)",
                     cd.getContactForce ().toString ("%.3f"));
            contacts.append (row + "\n");
            row = String.format ("%-20s%-6s", "FRICTION FORCE (N)",
                     cd.getFrictionForce ().toString ("%.3f"));
            contacts.append (row + "\n\n");
         });
      }
      else {
         contacts.append ("NO CONTACT DETECTED." + "\n\n");
      }
      return contacts.toString ();
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