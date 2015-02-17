# Created by Stefan Henneking on 02/16/2015
# CSE 6730 - Modeling & Simulation, Project 1

# Set title and axes
set title 'Average travel and waiting time: dependency on arrival rate'
set xrange[0 : 3.6]
set yrange[0 : 400]
set xtics(0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5) nomirror
set ytics(0, 50, 100, 150, 200, 250, 300, 350, 400)
set xlabel 'Arrival rate'
set ylabel 'Average travel/waiting time in seconds'

#set logscale y

set key left

# Set plot styles
set style line 1 lt 3 lw 1
set grid ytics ls 1

# Set output file
set term postscript eps enhanced color
set output "validation-wait-travel-time.eps"

# Plot data file
plot 	'data/travel-time.dat' with linespoints lt 1 lc 1 pt 5 ps 0.7 title "Average Travel Time",\
		'data/wait-time.dat' with linespoints lt 1 lc 2 pt 5 ps 0.7 title "Average Waiting Time"

# Clean gnuplot settings
unset xtics
unset ytics
unset xlabel
unset ylabel
#unset logscale y
unset key
unset grid
unset style
unset term
unset output