# BNO055_Madgwick_Filter
# BNO055 Madgwick Filter (Arduino)

## 📌 Description
This project demonstrates **sensor fusion** using the **Madgwick filter** with the **Adafruit BNO055 IMU**.  
Instead of relying on the BNO055’s built-in fusion, the algorithm is implemented manually to calculate **roll, pitch, and yaw** from raw accelerometer, gyroscope, and magnetometer data.  

## 🛠️ Features
- Interfaces with the **BNO055 sensor** over I²C.  
- Reads **accelerometer, gyroscope, and magnetometer** values.  
- Implements the **Madgwick filter** (gradient descent-based orientation estimation).  
- Outputs **roll, pitch, and yaw** angles in degrees.  
- Sampling rate fixed at **100 Hz** for stable updates.  
- Includes **safe normalization** to prevent divide-by-zero errors.  

## 📊 Applications
- Attitude and Heading Reference Systems (**AHRS**).  
- Drones, robotics, and self-balancing systems.  
- Educational projects on sensor fusion.  
- Real-time motion tracking.  

## 🚀 How It Works
1. **Gyroscope** provides smooth short-term rotation rates.  
2. **Accelerometer** gives gravity direction for long-term stability.  
3. **Magnetometer** adds heading (yaw) correction.  
4. **Madgwick filter** fuses all three using quaternion-based math + gradient descent correction.  
5. The final quaternion is converted into **roll, pitch, and yaw**.  

## 🔧 Requirements
- **Arduino IDE** (1.8.x or 2.x).  
- Libraries:  
  - `Adafruit_BNO055`  
  - `Adafruit_Sensor`  

## ▶️ Usage
1. Clone or download this repository.  
2. Open the `.ino` file in Arduino IDE.  
3. Install the required libraries from Library Manager.  
4. Upload the sketch to your Arduino board.  
5. Open the Serial Monitor at **115200 baud** to view:  


