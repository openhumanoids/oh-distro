import csv     # imports the csv module
import sys      # imports the sys module
import subprocess


inp = '../models/V1/urdf/V1_sim.urdf'


f = open("atlas_val_mappings.csv", 'rb') # opens the csv file

try:
    reader = csv.reader(f)  # creates the reader object
    for row in reader:   # iterates the rows of the file in orders

        row[0]= row[0].replace('/', "\/")

        rename_cmd = str('s/>' + row[0] + '</>' + row[1] + '</')
        print row[0] , ' to '  , row[1], ': ' , rename_cmd
        sub = subprocess.call(['sed','-i',rename_cmd, inp])

        rename_cmd = str('s/"' + row[0] + '"/"' + row[1] + '"/')
        print row[0] , ' to '  , row[1], ': ' , rename_cmd
        sub = subprocess.call(['sed','-i',rename_cmd, inp])

        #q = z


finally:
    f.close()      # closing
