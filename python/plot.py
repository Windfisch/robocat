import matplotlib.pyplot as plt
import sys

if len(sys.argv) <= 1 or sys.argv[1] in { '-h', '--help' }:
	print(f"Usage: {sys.argv[0]} tab-separated-file.txt [column for x axis] [column for 1st y axis] [column for 2nd y axis] [...]")
	print(f"       {sys.argv[0]} tab-separated-file.txt [COLUMNS]")
	print( "       Column indices must be given as zero-based integers, i.e. 0 for the first column.")
	print( "       If no column indices are given, '0 1 2 3 4 5 ...' is assumed, i.e. all columns are plotted against the first")
	print( "       Columns can also be given without spaces, i.e. '012345' instead of '0 1 2 3 4 5'")
	exit(0)

filename = sys.argv[1]

if len(sys.argv[2:]) >= 1:
	if len(sys.argv[2:]) == 1:
		axes = [int(c) for c in sys.argv[2]]
	else:
		axes = [int(c) for c in sys.argv[2:]]
else:
	axes = None

file = open(filename, "r")

header = file.readline()

columns = header.split('\t')

if axes is None:
	axes = list(range(len(columns)))

#timecolumn = columns[0]
#datacolumns = columns[1:]

timecolumn = columns[axes[0]]
datacolumns = [columns[i] for i in axes[1:]]

n=len(datacolumns)

data = { name: [] for name in datacolumns }

thin = 1
for (i,line) in enumerate(file):
	if i % thin != 0: continue

	values = line.split('\t')
	time = values[axes[0]]
	values = [values[i] for i in axes[1:]]

	for name, val in zip(datacolumns, values):
		data[name].append((float(time), float(val)))

ax1 = plt.subplot(n,1,1)

for (i,col) in enumerate(datacolumns):
	plt.subplot(n,1,i+1, sharex=ax1)
	d = data[col]
	plt.plot([t for (t,v) in d], [v for (t,v) in d])
	plt.ylabel(col)

plt.show()
