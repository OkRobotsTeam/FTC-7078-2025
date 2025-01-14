package org.firstinspires.ftc.teamcode;
import fi.iki.elonen.NanoHTTPD;
import org.firstinspires.ftc.ftccommon.external.WebHandlerRegistrar;
import com.qualcomm.robotcore.util.WebHandlerManager;
import fi.iki.elonen.NanoHTTPD.Response;

import java.io.IOException;
import android.content.res.AssetManager;
import android.content.Context;
import java.util.Map;
import java.io.BufferedReader;
import java.io.FileReader;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.webserver.WebHandler;



public class LogViewer implements WebHandler {

    @WebHandlerRegistrar
    public static void attachWebServer(Context context, WebHandlerManager manager) {
        manager.register("/log",new LogViewer());
    }

    private String inputDefault(Map<String, String> parms, String name) {
        String output = "<input type='text' name='" + name + "' value='";
        if (parms.get(name) != null) {
            output += parms.get(name);
        }
        output += "'>";
        return output;
    }

    public Response getResponse(NanoHTTPD.IHTTPSession session) {
        String logFile = "";
        String msg = "<html><body><h1>Log Viewer</h1>\n";
        Map<String, String> parms = session.getParms();
        if (parms.get("green")== null) {
            parms.put("green","System.out");
        }
        if (parms.get("red")== null) {
            parms.put("red","ERROR");
        }
        msg += "<form action='?' method='get'>\n" + "  <p>\n";
        msg += "Green Search: " + inputDefault(parms, "green")+ "<p>\n";
        msg += "Red Search: " + inputDefault(parms, "red") + "<p>\n";
        msg += "All: <input type=checkbox name=all";
        if (parms.get("all") != null) {
            msg += " checked";
        }
        msg += "> Unless this is enabled, it will only show log entries since the last opmode was started.<p>\n";
        msg += "<input type=submit name='Go'></form>\n<pre>\n";
        String line;
        String add = "";
        boolean greenSearch = parms.get("green") != null;
        boolean redSearch = parms.get("red") != null;
        try (BufferedReader br = new BufferedReader(new FileReader("/mnt/runtime/write/emulated/0/robotControllerLog.txt"))) {
            while ((line = br.readLine()) != null) {
                if (redSearch && line.contains(parms.get("red"))) {
                    add += "<font style='background-color:pink'>"+line+"</font>\n";
                } else if (greenSearch && line.contains(parms.get("green"))) {
                    add += "<font style='background-color:aquamarine'>"+line+"</font>\n";
                } else {
                    add += line + "\n";
                }
                if (parms.get("all") == null ) {
                    if (line.contains("******************** START")) {
                        //if we aren't set to print the whole file, then every time we hit the start of an opmode, throw out everything in the log before now.
                        add = line;
                    }
                }
            }
        } catch (IOException e) {
            System.err.println("Error reading the file: " + e.getMessage());
        }
        msg += add + "</pre>\n";
        msg += "</body></html>\n";
        return NanoHTTPD.newFixedLengthResponse(Response.Status.OK, NanoHTTPD.MIME_HTML, msg);
    }
}

