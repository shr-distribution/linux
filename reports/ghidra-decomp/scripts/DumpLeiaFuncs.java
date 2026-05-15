// Dump decompilation of all leia_* functions to a text file.
// Run via: analyzeHeadless <project_dir> <project_name> -import <binary>
//          -postScript DumpLeiaFuncs.java
//          -scriptPath /tmp/ghidra-decomp

import ghidra.app.script.GhidraScript;
import ghidra.app.decompiler.DecompInterface;
import ghidra.app.decompiler.DecompileResults;
import ghidra.program.model.listing.Function;
import ghidra.program.model.listing.FunctionIterator;

import java.io.PrintWriter;
import java.io.FileWriter;

public class DumpLeiaFuncs extends GhidraScript {
    @Override
    public void run() throws Exception {
        DecompInterface ifc = new DecompInterface();
        ifc.openProgram(currentProgram);

        String binName = currentProgram.getName();
        String outPath = "/tmp/ghidra-decomp/" + binName + ".decomp.txt";

        try (PrintWriter out = new PrintWriter(new FileWriter(outPath))) {
            out.println("=== Decompilation of leia_* / *_context_* / *_preamble_* in " + binName + " ===");
            out.println();

            FunctionIterator it = currentProgram.getListing().getFunctions(true);
            int n = 0;
            while (it.hasNext() && !monitor.isCancelled()) {
                Function f = it.next();
                String name = f.getName();
                if (!name.startsWith("leia_") &&
                    !name.contains("context_create") &&
                    !name.contains("context_destroy") &&
                    !name.contains("preamble") &&
                    !name.contains("drawctxt"))
                    continue;

                out.println("// ========== " + name +
                            "  @ " + f.getEntryPoint() +
                            "  size=" + f.getBody().getNumAddresses() + " ==========");
                DecompileResults res = ifc.decompileFunction(f, 60, monitor);
                if (res != null && res.getDecompiledFunction() != null) {
                    out.println(res.getDecompiledFunction().getC());
                }
                out.println();
                n++;
                if (n % 10 == 0)
                    println("  ... decompiled " + n + " functions");
            }

            println("Decompiled " + n + " functions to " + outPath);
        }
    }
}
