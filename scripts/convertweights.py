def process_mem_file(input_file, output_file):
    with open(input_file, "r") as infile, open(output_file, "w") as outfile:
        for line_num, line in enumerate(infile, start=1):
            hex_str = line.strip()

            # Skip empty lines
            if not hex_str:
                continue

            try:
                # Convert hex string to integer
                value = int(hex_str, 16)

                # Ensure it is an 8-bit value
                if not (0 <= value <= 0xFF):
                    print(f"Line {line_num}: Skipping out-of-range value '{hex_str}'")
                    continue

                # Halve (rounded down)
                halved = value // 2

                # Convert back to uppercase 2-digit hex
                output_hex = f"{halved:02X}"

                # Duplicate each output line
                outfile.write(output_hex + "\n")
                outfile.write(output_hex + "\n")

            except ValueError:
                print(f"Line {line_num}: Skipping invalid hex value '{hex_str}'")


if __name__ == "__main__":
    input_filename = "weights/2048weights_q8_c3.mem"
    output_filename = "weights/4096weights_q8_c3.mem"

    process_mem_file(input_filename, output_filename)

    print(f"Done! Processed '{input_filename}' -> '{output_filename}'")