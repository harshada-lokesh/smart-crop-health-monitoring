!nvidia-smi
!unzip -q /content/data.zip -d /content/custom_data
!wget -O /content/train_val_split.py https://raw.githubusercontent.com/EdjeElectronics/Train-and-Deploy-YOLO-Models/refs/heads/main/utils/train_val_split.py

# TO DO: Improve robustness of train_val_split.py script so it can handle nested data folders, etc
!python train_val_split.py --datapath="/content/custom_data" --train_pct=0.9
!pip install Ultralytics
# Python function to automatically create data.yaml config file
# 1. Reads "classes.txt" file to get list of class names
# 2. Creates data dictionary with correct paths to folders, number of classes, and names of classes
# 3. Writes data in YAML format to data.yaml

import yaml
import os

def create_data_yaml(path_to_classes_txt, path_to_data_yaml):

  # Read class.txt to get class names
  if not os.path.exists(path_to_classes_txt):
    print(f'classes.txt file not found! Please create a classes.txt labelmap and move it to {path_to_classes_txt}')
    return
  with open(path_to_classes_txt, 'r') as f:
    classes = []
    for line in f.readlines():
      if len(line.strip()) == 0: continue
      classes.append(line.strip())
  number_of_classes = len(classes)

  # Create data dictionary
  data = {
      'path': '/content/data',
      'train': 'train/images',
      'val': 'validation/images',
      'nc': number_of_classes,
      'names': classes
  }

  # Write data to YAML file
  with open(path_to_data_yaml, 'w') as f:
    yaml.dump(data, f, sort_keys=False)
  print(f'Created config file at {path_to_data_yaml}')

  return

# Define path to classes.txt and run function
path_to_classes_txt = '/content/custom_data/classes.txt'
path_to_data_yaml = '/content/data.yaml'

create_data_yaml(path_to_classes_txt, path_to_data_yaml)

print('\nFile contents:\n')
!cat /content/data.yaml
!yolo detect train data=/content/data.yaml model=yolo11s.pt epochs=40 imgsz=640
!yolo detect predict model=runs/detect/train/weights/best.pt source=data/validation/images save=True
import glob
from IPython.display import Image, display
for image_path in glob.glob(f'/content/runs/detect/predict/*.jpg')[:10]:
  display(Image(filename=image_path, height=400))
  print('\n')
# Create "my_model" folder to store model weights and train results
!mkdir /content/my_model
!cp /content/runs/detect/train3/weights/best.pt /content/my_model/my_model.pt
!cp -r /content/runs/detect/train3 /content/my_model

# Zip into "my_model.zip"
%cd my_model
!zip /content/my_model.zip my_model.pt
!zip -r /content/my_model.zip train
%cd /content
import pandas as pd

# Load the results.csv file
results_path = '/content/runs/detect/train/results.csv'
df = pd.read_csv(results_path)

# Strip any whitespace from column names
df.columns = df.columns.str.strip()

# Print the column names
print("Column names in results.csv:")
print(df.columns.tolist())
import pandas as pd
import matplotlib.pyplot as plt

# Load the results.csv file
results_path = '/content/runs/detect/train3/results.csv'
df = pd.read_csv(results_path)

# Strip any whitespace from column names
df.columns = df.columns.str.strip()

# Extract epoch and metrics
epochs = df['epoch']
train_precision = df['metrics/precision(B)']  # Using precision as a proxy for training
val_map = df['metrics/mAP50(B)']             # Validation mAP@0.5

# Plot the graph
plt.figure(figsize=(10, 6))
plt.plot(epochs, train_precision, label='train (precision)', color='blue')
plt.plot(epochs, val_map, label='val (mAP@0.5)', color='orange')
plt.xlabel('epoch')
plt.ylabel('metrics')
plt.title('Training Precision and Validation mAP@0.5')
plt.legend()
plt.grid(True)

# Save and display the plot
plt.savefig('/content/metrics_vs_epoch.png')
plt.show()
