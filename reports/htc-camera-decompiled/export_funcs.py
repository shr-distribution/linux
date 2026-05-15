# Export decompiled functions
# @category Export
# @runtime Jython

from ghidra.app.decompiler import DecompInterface
from ghidra.util.task import ConsoleTaskMonitor
import os

def run():
    args = getScriptArgs()
    if len(args) < 1:
        output_path = "/tmp/ghidra_htc_camera/" + currentProgram.getName() + "_decompiled.c"
    else:
        output_path = args[0]
    
    decomp = DecompInterface()
    decomp.openProgram(currentProgram)
    monitor = ConsoleTaskMonitor()
    
    with open(output_path, 'w') as f:
        f.write("// Decompiled: " + currentProgram.getName() + "\n\n")
        
        fm = currentProgram.getFunctionManager()
        funcs = fm.getFunctions(True)
        count = 0
        for func in funcs:
            results = decomp.decompileFunction(func, 30, monitor)
            if results.decompileCompleted():
                dec_func = results.getDecompiledFunction()
                if dec_func:
                    f.write("\n// " + func.getName() + " @ " + str(func.getEntryPoint()) + "\n")
                    f.write(dec_func.getC() + "\n")
                    count += 1
        
        print("Exported %d functions to %s" % (count, output_path))
    
    decomp.dispose()

run()
