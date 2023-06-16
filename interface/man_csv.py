"""
This module provides functions for converting angles between radians and
degrees, as well as saving and loading DataFrames to/from CSV files.

Functions:
    - rad2deg(df): Convert values in a DataFrame from radians to degrees.
    - deg2rad(df): Convert values in a DataFrame from degrees to radians.
    - save_csv(df, name): Save a DataFrame to a CSV file.
    - load_csv(name): Load a DataFrame from a CSV file.
    - main(): Main function demonstrating the usage of the provided functions.
"""


from pathlib import Path
import numpy as np
import pandas as pd


def rad2deg(df):
    for c in df.columns:
        df[c] = np.rad2deg(df[c])


def deg2rad(df):
    for c in df.columns:
        df[c] = np.deg2rad(df[c])


def save_csv(df, name):
    df.to_csv(Path(name).with_suffix(".csv"), index=False)


def load_csv(name):
    return pd.read_csv(Path(name).with_suffix(".csv"))


def main() -> None:
    df = pd.DataFrame({"a": [3.14, 2 * 3.14]})
    save_csv(df, "rad")
    rad2deg(df)
    save_csv(df, "deg")


if __name__ == "__main__":
    main()
