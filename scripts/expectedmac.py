def sum_squared_hex_values(input_file):
    total = 0

    with open(input_file, "r") as infile:
        for line_num, line in enumerate(infile, start=1):
            hex_str = line.strip()
            if not hex_str:
                continue

            try:
                value = int(hex_str, 16)
                if not (0 <= value <= 0xFF):
                    print(
                        f"Line {line_num}: "
                        f"Skipping out-of-range value '{hex_str}'"
                    )
                    continue

                total += value * value

            except ValueError:
                print(
                    f"Line {line_num}: "
                    f"Skipping invalid hex value '{hex_str}'"
                )

    return total


if __name__ == "__main__":
    input_filename = "weights/4096weights_q8_c3.mem"

    result = sum_squared_hex_values(input_filename)

    print(f"Sum of squared values: {result}")
    print(f"Hex: 0x{result:X}")