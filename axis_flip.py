import pandas as pd
df = pd.read_csv("Setup-B.csv")

# --- Coordinate transform ---
# Correct reflection: x_new = 2.4 - x_old, y_new = -4.8 - y_old
df["drone_x"] = (2.4 - df["drone_x"]).round(3)
df["drone_y"] = (-4.8 - df["drone_y"]).round(3)

# --- MAC modification ---
def modify_mac(mac):
    mac = str(mac)
    if mac.endswith("3"):
        return mac[:-1] + "4"
    if mac.endswith("B"):
        return mac[:-1] + "C"
    return mac

df["anchor_mac"] = df["anchor_mac"].apply(modify_mac)
df.to_csv("Setup-B_converted.csv", index=False)
print("Done. Clean values saved.")