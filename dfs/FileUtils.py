import csv

def writeCoordsToCSVFile(coords):
    csv_file = open("DFS.txt", "w+", newline='')
    csv_writer = csv.writer(csv_file)

    for (x,y) in coords:
        csv_writer.writerow([x,y])

    csv_file.close()