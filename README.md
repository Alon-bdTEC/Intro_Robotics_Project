# Scanning Robotic Arm Project (In Nuclear Waste Storage)

## Project Overview
This project focuses on a 6R serial robotic arm (FANUC model M-10iA) mounted on a rail, designed to scan barcodes of nuclear waste storage containers in a storage corridor.

## Repository Contents

### Documentation
- `ProjectEnglishVR.pdf`: Comprehensive project report in English
- `PDF_Project_Code.pdf`: Detailed code documentation
- `README.md`: Project overview and repository guide

### Code
- `Project_Code.m`: MATLAB implementation of the robotic arm trajectory and analysis

### Visualizations
- `Combined_Abs.jpg`: Absolute motion plot (relative to initial ground position)
- `Combined_Rel.jpg`: Relative motion plot (relative to the moving robot base)

### Motion Demonstration Videos
- `my_video_abs.mp4`: Absolute motion video (relative to initial ground position)
- `my_video_rel.mp4`: Relative motion video (relative to the moving robot base)

## Motion Visualization - Images and Videos

### Absolute Movement
**Plot:**
![Combined Absolute Movement](Combined_Abs.jpg)

**Video Demonstration:**
<video width="640" height="480" controls>
  <source src="my_video_abs.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>

### Relative Movement
**Plot:**
![Combined Relative Movement](Combined_Rel.jpg)

**Video Demonstration:**
<video width="640" height="480" controls>
  <source src="my_video_rel.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>

### Visualization Details
- Both plots and videos show the robot's motion with a time step of dt = 0.01 [sec]
- Demonstrates precise trajectory and barcode scanning motion
- Videos provide continuous motion representation
