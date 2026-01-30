import pandas as pd

# === PUT YOUR CSV FILE PATHS HERE ===
csv_files = [
    r"C:\Users\aryas\Desktop\AoA_JAN\B-LowLevel",
    r"C:\Users\aryas\Desktop\AoA_JAN\B-MidLevel",
    r"C:\Users\aryas\Desktop\AoA_JAN\B-HighLevel"
]
# Output file
output_file = "Setup-B"

# Expected column order
COLUMNS = [
    "anchor_mac",
    "azimuth",
    "channel",
    "drone_x",
    "drone_y",
    "drone_z",
    "elevation",
    "rssi",
    "tag_mac",
    "time"
]

dataframes = []

for file in csv_files:
    try:
        print(f"📄 Reading: {file}")
        df = pd.read_csv(file)

        # Keep only required columns and in the correct order
        df = df[COLUMNS]

        dataframes.append(df)

    except Exception as e:
        print(f"⚠️ Skipping {file} due to error: {e}")

if not dataframes:
    print("❌ No valid CSV files were loaded.")
else:
    combined_df = pd.concat(dataframes, ignore_index=True)

    combined_df.to_csv(output_file, index=False)

    print(f"\n✅ Combined {len(dataframes)} files into '{output_file}'")
    print(f"📊 Total rows: {len(combined_df)}")
