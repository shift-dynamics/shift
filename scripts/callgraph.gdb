# From the build folder, run as:
# gdb --command=../scripts/callgraph.gdb 
#     --args ./bin/shift-cli -i <json_input_file>
#
# Warning, this will generate very verbose output 
set pagination off
set logging file callgraph.txt
set logging on
rbreak Context.cc:.*
rbreak Jacobian.cc:.*
rbreak RNCA.cc:.*
rbreak ODE113.cc:.*
rbreak Substructure.cc:.*
command 1-1000
  backtrace
  continue
end
run
set logging off
quit
