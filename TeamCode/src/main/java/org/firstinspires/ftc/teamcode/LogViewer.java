package org.firstinspires.ftc.teamcode;
import fi.iki.elonen.NanoHTTPD;
import org.firstinspires.ftc.ftccommon.external.WebHandlerRegistrar;
import com.qualcomm.robotcore.util.WebHandlerManager;
import fi.iki.elonen.NanoHTTPD.Response;

import java.io.File;
import java.io.IOException;
import android.content.res.AssetManager;
import android.content.Context;
import android.util.Pair;

import java.util.ArrayList;
import java.util.Date;
import java.util.Map;
import java.io.BufferedReader;
import java.io.FileReader;
import java.util.Timer;
import java.util.stream.Collectors;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.webserver.WebHandler;



public class LogViewer implements WebHandler {

    @WebHandlerRegistrar

    public static void attachWebServer(Context context, WebHandlerManager manager) {
        File fileDir = context.getFilesDir();
        LogViewer lv = new LogViewer();
        manager.register("/log",lv);
        manager.register("/logviewer",lv);
        manager.register("/logs",lv);
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
        long startTime = System.currentTimeMillis();
        String logFile = "";
        ArrayList<String> html = new ArrayList<String>();

        html.add("<html><body><h1>Log Viewer</h1>\n");
        Map<String, String> parms = session.getParms();
//        if (parms.get("red")== null) {
//            parms.put("red","ERROR");
//        }

        html.add("<form action='?' method='get'>\n" + "  <p>\n");
        html.add("Red Search: " + inputDefault(parms, "red") + "<p>\n");
        html.add("Start: " + inputDefault(parms, "start") + "<p>\n");
        html.add("End: " + inputDefault(parms, "end") + "<p>\n");
        html.add("All: <input type=checkbox name=all");
        if (parms.get("all") != null) {
            html.add(" checked");
        }
        html.add("> Unless this is enabled, it will only show log entries since the last opmode was started.<p>\n");
        html.add("<input type=submit name='Go'></form>\n<pre>\n");
        String line;
        ArrayList<String> logLines = new ArrayList<String>();

        boolean redSearch = (parms.get("red") != null) && (!parms.get("red").isEmpty());

        String logDir = "/storage/emulated/0";
        try {
            File f = new File(logDir +"/robotControllerLog.txt" );
            if (!f.canRead()) {
                File d = new File(logDir);
                if (!d.exists())
                    html.add("No File/Dir " + logDir + "\n");
                if (d.isDirectory()) {
                    html.add("Files in " + logDir + "<p>\n");
                    for (File file : d.listFiles()) {
                        Date date = new Date(file.lastModified());
                        html.add(file.getName() + " : " + file.length() + " : " + date + "<p>\n");
                    }
                } else {
                    html.add("Not directory " + logDir + "\n");
                }
            }
            BufferedReader br = new BufferedReader(new FileReader(logDir+"/robotControllerLog.txt"));
            ArrayList<String> lines = new ArrayList<String>();
            while ((line = br.readLine()) != null) {
                lines.add(line);
            };
            int startLine = 0;
            int endLine = 0;
            ArrayList<Integer> opModeStarts = new ArrayList<Integer>();
            if (parms.get("all") == null ) {
                html.add("Seeking last opmode start\n");
                for (int i = 0; i< lines.size() ; i++) {
                    if (lines.get(i).contains("******************** START")) {
                        opModeStarts.add(i);
                    }
                }
                for (int i=0; i<opModeStarts.size()-1;i++) {
                    int l = opModeStarts.get(i);
                    int nl = opModeStarts.get(i+1);
                    html.add("Found: <a href=\"?start=" + l + "&end=" + nl + "\">" + l + "</a>:" + lines.get(l)+"\n");
                }
                if (opModeStarts.size() > 0) {
                    int l = opModeStarts.get(opModeStarts.size()-1);
                    html.add("Found: <a href=\"?start=" + l + "\">" + l + "</a>:" + lines.get(l)+"\n");
                }
            }
            if (parms.get("start") != null) {

                startLine = Integer.parseInt(parms.get("start"));
                html.add("Overriding seek to start at line :"+startLine+"\n");
            }
            if (parms.get("end") != null) {
                endLine = Integer.parseInt(parms.get("end"));
                html.add("Ending at line :"+endLine+"\n");
            }
            if (endLine == 0) {
                endLine = lines.size()-1;
            }
            for (int i = startLine; i<=endLine; i++) {
                if (redSearch && lines.get(i).contains(parms.get("red"))) {
                    logLines.add("<font style='background-color:pink'>" + lines.get(i) + "</font>\n");
                } else if (lines.get(i).contains("********************")) {
                    logLines.add("<font style='background-color:red'>" + lines.get(i) + "</font>\n");
                } else if (lines.get(i).contains("System.out:")) {
                    logLines.add(lines.get(i) + "\n");
                } else {
                    logLines.add("<font style='background-color:aquamarine'>"+lines.get(i)+"</font>\n");
                }
//                if (parms.get("all") == null ) {
//                    if (line.contains("******************** START")) {
//                        //if we aren't set to print the whole file, then every time we hit the start of an opmode, throw out everything in the log before now.
//                        logLines.clear();
//                        logLines.add(line);
//                    }
//                }
            }
        } catch (IOException e) {
            html.add("Error reading the file: " + e.getMessage());
            System.err.println("Error reading the file: " + e.getMessage());
        }
        if (parms.get("delete") == "yes") {
            File d = new File(logDir);
            if (d.isDirectory()) {
                for (File file : d.listFiles()) {
                    if (file.getName() == "robotControllerLog.txt") {
                        //file.delete();
                        logLines.add("I should grr delete this file "+ file.getName() + "<p>\n");
                    }
                }
            }
        }
        logLines.add("generating page took " + (System.currentTimeMillis() - startTime) + " milliseconds\n");
        //html.add(logLines.stream().collect(Collectors.joining("")) + "</pre>\n");
        html.addAll(logLines);
        html.add("</pre>\n");
        html.add("</body></html>\n");

        return NanoHTTPD.newFixedLengthResponse(Response.Status.OK, NanoHTTPD.MIME_HTML, html.stream().collect(Collectors.joining("")));
    }
}

