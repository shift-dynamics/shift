# From the build folder, run as:
# gdb --command=../scripts/callgraph.gdb \
#     --args ./bin/shift-cli -i <json_input_file>
#
# Warning, this will generate very verbose output! 
set pagination off
set logging file callgraph.txt
set logging on
rbreak context.cc:.*
rbreak jacobian.cc:.*
rbreak rnca.cc:.*
rbreak ode113.cc:.*
rbreak substructure.cc:.*
command 1-1000
  backtrace
  continue
end
run
set logging off
quit
