import ghidra.app.script.GhidraScript;
import ghidra.app.decompiler.DecompInterface;
import ghidra.app.decompiler.DecompileResults;
import ghidra.program.model.listing.Function;
import ghidra.program.model.listing.FunctionIterator;
import java.io.PrintWriter;
import java.io.FileWriter;
public class DumpLeiaFuncsTagged extends GhidraScript {
    @Override
    public void run() throws Exception {
        DecompInterface ifc = new DecompInterface();
        ifc.openProgram(currentProgram);
        String tag = System.getenv("DECOMP_TAG");
        if (tag == null) tag = "untagged";
        String binName = currentProgram.getName();
        String outPath = "/tmp/ghidra-decomp/" + tag + "_" + binName + ".decomp.txt";
        try (PrintWriter out = new PrintWriter(new FileWriter(outPath))) {
            out.println("=== Decompilation of leia_*/context_*/preamble_*/drawctxt in " + tag + ":" + binName + " ===");
            FunctionIterator it = currentProgram.getListing().getFunctions(true);
            int n = 0;
            while (it.hasNext() && !monitor.isCancelled()) {
                Function f = it.next();
                String name = f.getName();
                if (!name.startsWith("leia_") &&
                    !name.contains("context_create") && !name.contains("context_destroy") &&
                    !name.contains("preamble") && !name.contains("drawctxt") &&
                    !name.contains("set_hw_") && !name.contains("perform_resolve") &&
                    !name.contains("emit_") && !name.contains("_state_") &&
                    !name.contains("draw_init") && !name.contains("setup_"))
                    continue;
                out.println("// ========== " + name + "  @ " + f.getEntryPoint() +
                            "  size=" + f.getBody().getNumAddresses() + " ==========");
                DecompileResults res = ifc.decompileFunction(f, 60, monitor);
                if (res != null && res.getDecompiledFunction() != null)
                    out.println(res.getDecompiledFunction().getC());
                out.println();
                n++;
                if (n % 20 == 0) println("  ... " + n);
            }
            println("Decompiled " + n + " functions to " + outPath);
        }
    }
}
