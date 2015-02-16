# Created by Stefan Henneking on 01/29/2015

# Set title and axes
set title 'Exponential Distribution: Inter-Arrival Times'
set xrange[-100:10100]
set yrange[0:55]
set xtics(0, 2000, 4000, 6000, 8000, 10000)
set ytics(0, 5, 10, 20, 30, 40, 50)
set xlabel 'Vehicle IDs (sorted by IAT)'
set ylabel 'Inter-Arrival Time'

# Set plot styles
set style line 1 lt 3 lw 1
set grid ytics ls 1

# Set output file
set term postscript eps enhanced color
set output "data.eps"

# Plot data file
plot '< sort -n -r -k2 inter-arrival-time.dat' u 2 with points ps 0.4 pt 5 notitle

# Clean gnuplot settings
unset xrange
unset yrange
unset xtics
unset ytics
unset xlabel
unset ylabel
unset logscale x
unset logscale y
unset grid
unset style
unset term
unset output