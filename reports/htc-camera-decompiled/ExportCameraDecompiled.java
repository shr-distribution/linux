//Export decompiled C code for all functions to specified output
//@category Export
//@keybinding
//@menupath
//@toolbar

import ghidra.app.script.GhidraScript;
import ghidra.app.decompiler.*;
import ghidra.program.model.listing.*;
import ghidra.util.task.TaskMonitor;
import java.io.*;

public class ExportCameraDecompiled extends GhidraScript {

    @Override
    public void run() throws Exception {
        String[] args = getScriptArgs();
        String outputFile;

        if (args.length > 0) {
            outputFile = args[0];
        } else {
            outputFile = "/tmp/ghidra_htc_camera/" + currentProgram.getName() + "_decompiled.c";
        }

        DecompInterface decompiler = new DecompInterface();
        decompiler.openProgram(currentProgram);

        DecompileOptions options = new DecompileOptions();
        decompiler.setOptions(options);

        FunctionManager fm = currentProgram.getFunctionManager();
        FunctionIterator functions = fm.getFunctions(true);

        PrintWriter writer = new PrintWriter(new FileWriter(outputFile));

        writer.println("/* Decompiled from " + currentProgram.getName() + " using Ghidra */");
        writer.println("/* HTC EV Shooter Android ROM - Camera Binaries */");
        writer.println();

        int count = 0;
        int total = 0;

        FunctionIterator countIter = fm.getFunctions(true);
        while (countIter.hasNext()) {
            Function f = countIter.next();
            if (!f.isExternal() && !f.isThunk()) {
                total++;
            }
        }

        println("Starting decompilation of " + total + " functions from " + currentProgram.getName());

        while (functions.hasNext()) {
            Function func = functions.next();

            if (func.isExternal() || func.isThunk()) {
                continue;
            }

            try {
                DecompileResults results = decompiler.decompileFunction(func, 60, monitor);

                if (results != null && results.decompileCompleted()) {
                    DecompiledFunction decomp = results.getDecompiledFunction();
                    if (decomp != null) {
                        String cCode = decomp.getC();
                        if (cCode != null && !cCode.isEmpty()) {
                            writer.println("/* ============================================= */");
                            writer.println("/* Function: " + func.getName() + " */");
                            writer.println("/* Address: 0x" + func.getEntryPoint().toString() + " */");
                            writer.println("/* ============================================= */");
                            writer.println(cCode);
                            writer.println();
                            count++;
                        }
                    }
                }
            } catch (Exception e) {
                println("Error decompiling " + func.getName() + ": " + e.getMessage());
            }

            if (count % 50 == 0 && count > 0) {
                println("Progress: " + count + "/" + total + " functions decompiled");
            }
        }

        writer.close();
        decompiler.dispose();

        println("Decompilation complete!");
        println("Total functions decompiled: " + count);
        println("Output written to: " + outputFile);
    }
}
