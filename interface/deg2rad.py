import sys

from man_csv import deg2rad, load_csv, save_csv


def main():
    # Check if two arguments are provided
    if len(sys.argv) != 3:
        print("De o nome de entrada e de saida dos arquivos")
        return

    # Get the strings from command-line arguments
    input_name = sys.argv[1]
    output_name = sys.argv[2]

    df = load_csv(input_name)
    deg2rad(df)
    save_csv(df, output_name)


if __name__ == "__main__":
    main()
