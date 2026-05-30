BLOCK_SIZE = 256
filenum = 2

def process_mem_file(input_file, output_file):
    with open(input_file, "r") as infile:
        # Read non-empty lines
        lines = [line.strip() for line in infile if line.strip()]

    with open(output_file, "w") as outfile:
        # Process file in 256-byte blocks
        for block_start in range(0, len(lines), BLOCK_SIZE):
            block = lines[block_start:block_start + BLOCK_SIZE]

            parsed_values = []

            # Parse and validate block
            for line_num, hex_str in enumerate(block, start=block_start + 1):
                try:
                    value = int(hex_str, 16)

                    if not (0 <= value <= 0xFF):
                        print(
                            f"Line {line_num}: "
                            f"Skipping out-of-range value '{hex_str}'"
                        )
                        continue

                    parsed_values.append(value)

                except ValueError:
                    print(
                        f"Line {line_num}: "
                        f"Skipping invalid hex value '{hex_str}'"
                    )

            # 1. Write original 256 values

            # 2. Write first halved copy
            for value in parsed_values:
                outfile.write(f"{int(value / 1.41):02X}\n")

            # 3. Write second halved copy
            for value in parsed_values:
                outfile.write(f"{int(value / 1.41):02X}\n")


if __name__ == "__main__":
    input_filename = "weights/2048weights_q8_c0.mem"
    output_filename = "weights/4096weights_q8_c0.mem"

    process_mem_file(input_filename, output_filename)

    print(f"Done! Processed '{input_filename}' -> '{output_filename}'")