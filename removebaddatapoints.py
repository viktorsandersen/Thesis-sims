#!python3
# This script removes bad data points from the data files.
import pandas as pd

# Load the data
df = pd.read_csv("accFF_condim1_friction1_dampening1.csv")

# Remove bad data points
df = df[df["force_reference_2"] != 0]

# Save the cleaned data
df.to_csv("accFF_condim1_friction1_dampening1_cleaned.csv", index=False)