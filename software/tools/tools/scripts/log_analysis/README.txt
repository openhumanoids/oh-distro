About these tools:
log_analysis.py reads lcmlog and prints text file with different formats, which are processed as follows:

mode	processed by & purpose
0	make_raw_activity.m basic plot of when messages were published
1	make_gantt.m gantt chart. only works for drill task
1	make_activity.m creates plot of the fraction of time spent moving (and walking)
2&3	make_bw.m analysis of bandwidth from BASE_BW_STATS
	make_plot.m runns them all for one combined plot
