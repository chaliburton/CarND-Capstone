import csv
import xml.etree.cElementTree as ET
import glob

# Path to xml file
data_path = '../TrainData/Carla_rosbag/'

# Path to csv file
csv_path = data_path

# csv filename
csv_file_name = csv_path + 'label_training.csv'

# Open CSV file for writing
csv_file = open(csv_file_name,'w')
# create csv writer
Csv_writer = csv.writer(csv_file)

# write header
Csv_writer.writerow(['frame', 'xmin', 'xmax', 'ymin', 'ymax', 'class_id'])

# Read all xml file names in dir
for xmlfile in glob.glob(data_path+'*.xml'):
    # Load XML file
    tree = ET.parse(xmlfile)
    root = tree.getroot()

    imgfile = root.find('filename').text
    print(imgfile)


    for obj in root.iter('object'): 
        classId = obj.find('name').text
        # convert to number
        if classId.capitalize()=='Green':
            print('green found')
            classIdNr = 1
        elif classId.capitalize()=='Yellow':
            print('yellow found')
            classIdNr = 2
        elif classId.capitalize()=='Red':
            print('red found')
            classIdNr = 3
        else:
            print('unknown found: ',classId.capitalize())
            classIdNr = 0

        xmin = obj.find('./bndbox/xmin').text
        ymin = obj.find('./bndbox/ymin').text
        xmax = obj.find('./bndbox/xmax').text
        ymax = obj.find('./bndbox/ymax').text
        
        #print('xmin: ', obj.find('./bndbox/xmin').text)

        # write csv line
        Csv_writer.writerow([imgfile, xmin, xmax, ymin, ymax, classIdNr])

# close file
csv_file.close()