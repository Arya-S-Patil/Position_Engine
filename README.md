# Position Engine

Position Engine is a system for recording, processing, and analyzing Angle-of-Arrival (AoA) data using a multi-anchor setup. This repository contains scripts, datasets, and utilities for collecting AoA packets from four anchors and preparing them for machine learning and localization analysis.

---

## Overview

This project captures AoA measurements (**azimuth, elevation, RSSI**) from four synchronized anchors and converts them into structured datasets. It provides tools for:

- Combining CSV files from multiple anchors  
- Balancing data across anchors  
- Analyzing angle errors  
- Running UDP-based data capture  
- Preparing machine-learning-ready datasets  

The system is designed for:

- 3D localization experiments  
- AoA calibration and performance evaluation  
- Machine learning models for position estimation  

---

## Repository Structure

```

Position_Engine
├── Experimental Data/
│   └── Raw and processed angle measurement files
├── Complete_dataset.csv
├── Setup-A.csv
├── Setup-A-Per-Point-Anchor-Errors.csv
├── Setup-B.csv
├── Setup-B-converted.csv
├── Setup-B-Per-Point-Anchor-Errors.csv
├── angle_error_analysis.py
├── axis_flip.py
├── csv_combiner.py
├── main.py
├── ml_ready_maker.py
├── udp.py
├── index.html
├── fluxquery.txt
└── README.md

````

### File and Folder Descriptions

| File / Folder | Purpose |
|---------------|---------|
| `Experimental Data/` | Raw measurement logs and intermediate CSVs from anchor runs |
| `Complete_dataset.csv` | Combined dataset containing all anchors and positions |
| `Setup-A.csv`, `Setup-B.csv` | Per-setup measurement CSV files |
| `Setup-A/B-Per-Point-Anchor-Errors.csv` | Error metrics computed per anchor at each position |
| `csv_combiner.py` | Merges separate anchor CSV files into one dataset |
| `ml_ready_maker.py` | Balances data across anchors and produces ML-ready fused rows |
| `angle_error_analysis.py` | Computes azimuth and elevation error statistics |
| `axis_flip.py` | Corrects coordinate or angle axis mismatches |
| `udp.py` | UDP listener used to collect real-time anchor packets |
| `main.py` | Core script for running capture or processing pipelines |
| `fluxquery.txt` | Reference query used for exporting or filtering data |
| `index.html` | Basic HTML interface for local visualization |

---

## How to Use

### 1. Capture Data
Run the UDP listener to collect AoA packets:
(with all the visualization and Influx_DB database)
```bash
python main.py
````

### 2. Combine CSV Files

Merge per-anchor CSV files into a unified dataset:

```bash
python csv_combiner.py
```

### 3. Prepare Machine Learning Dataset

Balance all four anchors per `(x, y, z)` location and generate fused training data:

```bash
python ml_ready_maker.py
```

### 4. Analyze Angle Errors

Compute per-anchor error statistics:

```bash
python angle_error_analysis.py
```

---

## Balanced Dataset Concept

For each drone position `(drone_x, drone_y, drone_z)`, the system uses the **minimum number of packets recorded across all four anchors**. This ensures that each anchor contributes equally and prevents bias in machine learning models.

Each row in the ML-ready dataset represents one synchronized measurement set from all four anchors.

---

## Example ML Dataset Format

```
drone_x, drone_y, drone_z,
anchor1_azimuth, anchor1_elevation, anchor1_rssi,
anchor2_azimuth, anchor2_elevation, anchor2_rssi,
anchor3_azimuth, anchor3_elevation, anchor3_rssi,
anchor4_azimuth, anchor4_elevation, anchor4_rssi
```

---

## Requirements

Python 3.x is required.

Install dependencies:

```bash
pip install pandas numpy
```

---

## Contributing

Contributions and suggestions are welcome. Feel free to open issues or submit pull requests to improve data processing, visualization, or hardware support.

---

## License

Add your chosen license here (for example, MIT License).

---

## Author

**Arya S. Patil**

---
