# 🌿 Smart Crop Health Monitoring System
**A Hybrid IoT and Deep Learning Approach for Precision Agriculture**

[![Python 3.9+](https://img.shields.io/badge/python-3.9+-blue.svg)](https://www.python.org/downloads/)

This project presents an end-to-end smart farming solution. It combines **IoT soil sensing** using ESP32 with **Computer Vision** (YOLOv11 & CNN) to detect and classify diseases in crops like Tomato, Potato, and Chili.

---

## 📂 Project Structure

- **`Dataset/`**: Manually annotated image datasets for YOLO (Detection) and CNN (Classification).
- **`Graphs/`**: Training metrics including Confusion Matrices, P-R Curves, and Loss graphs.
- **`Hardware/`**: Arduino (.ino) code for NPK, DHT11, DS18B20, and Soil Moisture sensors.
- **`Models/`**: `my_model.pt` (YOLOv11) and `model.h5` (CNN - weights hosted on Hugging Face).
- **`Scripts/`**: Python scripts for model training and real-time webcam inference.

---

## 🤖 Machine Learning Performance

### YOLOv11 Detection (Real-Time)
Used for localized disease detection across 7+ crop varieties.
- **Results:** Precision, Recall, and mAP curves are stored in `/Graphs`.
- **Demo:**
![YOLO Detection]


### CNN Classification
A high-accuracy model for identifying healthy vs. diseased leaves.
- **Validation Accuracy:** 91%
- **Training Accuracy:** 97%
- **Metric:**
![CNN Confusion Matrix](Graphs/cnn_confusion_matrix.png)

---

## 🛠 Hardware Integration
The system uses an **ESP32** microcontroller to fetch real-time data from:
- **Atmospheric:** DHT11 (Temperature & Humidity)
- **Soil:** NPK Sensor (Nutrients), DS18B20 (Temperature), Capacitive Moisture Sensor.
- **IoT Dashboard:** Real-time monitoring via the **Blynk Mobile App**.



---

## 🚀 Getting Started

### Installation
1. Clone the repository:
   ```bash
   git clone [[https://github.com/YOUR_USERNAME/smart-crop-monitoring.git](https://github.com/YOUR_USERNAME/smart-crop-monitoring.git](https://github.com/harshada-lokesh/smart-crop-health-monitoring.git)]
   cd smart-crop-monitoring
