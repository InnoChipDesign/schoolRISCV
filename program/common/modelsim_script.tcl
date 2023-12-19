
# create library
vlib work

# compile sources
vlog \
     +incdir+../../../src \
     +incdir+../../../testbench \
     ../../../src/*.v \
     ../../../src/*.sv \
     ../../../testbench/*.v

# run simulator
vsim -voptargs="+acc" work.sm_testbench

# add signals to waveform
add wave -radix hex sim:/sm_testbench/sm_top/sm_cpu/*
add wave -radix hex sim:/sm_testbench/sm_top/sm_cpu/rf/*
add wave -radix hex sim:/sm_testbench/sm_top/sm_cpu/rf/rf

# start simulation
run -all

# Zoom Full the Wave window
wave zoom full
