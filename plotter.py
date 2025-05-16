import pandas as pd
import matplotlib.pyplot as plt
import argparse

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--file", type=str, required=True, help="Path to whitespace-delimited CSV file")
    args = parser.parse_args()

    # Load the CSV with whitespace as delimiter
    df = pd.read_csv(args.file, delim_whitespace=True)

    # Check for required columns
    required_cols = [f'actual_q_{i}' for i in range(6)] + [f'target_q_{i}' for i in range(6)] + ['timestamp']
    for col in required_cols:
        if col not in df.columns:
            raise ValueError(f"Missing required column: {col}")

    time = df['timestamp']
    joint_names = ['Base', 'Shoulder', 'Elbow', 'Wrist 1', 'Wrist 2', 'Wrist 3']

    plt.figure(figsize=(12, 10))
    for i in range(6):
        plt.subplot(3, 2, i + 1)
        plt.plot(time, df[f'target_q_{i}'], 'r--', label='Target')
        plt.plot(time, df[f'actual_q_{i}'], 'b-', label='Actual')
        plt.title(f'Joint {i} - {joint_names[i]}')
        plt.xlabel('Time (s)')
        plt.ylabel('Position (rad)')
        plt.legend()
        plt.grid(True)

    plt.tight_layout()
    plt.suptitle("Joint Positions (Target vs Actual)", y=1.02)
    plt.show()

if __name__ == "__main__":
    main()
