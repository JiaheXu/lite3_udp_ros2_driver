import re

def swap_word_endian(word):
    """Swap byte order of a 16-bit hex word (e.g., '0100' -> '0001')."""
    return word[2:] + word[:2]

# Command dictionary
CMD_MAP = {
    "21010c02": "manual mode",
    "21040001": "heartbeat",
    "21010202": "stand/sit",
    "21010131": "x,y move",
    "21010130": "pitch move",
    "21010102": "height move",
    "21010135": "yaw move",    

    "21010508": "motion1",

    "21010204": "shake body",

    "2101030c": "space step",    
    "21010c0b": "stop",       
}
# 21010507 say_hello (in static state)
# 21010204 shake body
# 2101030c space step(in static state)
# 2101020d turning jump

# 2101020d move after space
# 21010521 move after space
# 2101050b jump forward

# 21010522 last motion

def strip_trailing_zeros(words):
    """Remove trailing '0000' words from the list."""
    while words and words[-1] == "0000":
        words.pop()
    return words

def parse_txyx_file(input_file, output_file="parsed_cmds.txt"):
    results = []
    with open(input_file, "r") as f:
        lines = f.readlines()

    current_ts = None
    hex_data = []

    def process_packet(ts, hex_data):
        """Process one packet, return parsed line or None if skipped."""
        last_words = hex_data[-6:]
        last_words = [swap_word_endian(w) for w in last_words]

        if len(last_words) >= 2:
            # Swap first two and combine
            last_words[0], last_words[1] = last_words[1], last_words[0]
            combined = last_words[0] + last_words[1]
            last_words = [combined] + last_words[2:]

            # Skip heartbeat
            if combined.lower() in [ "21040001", "21010131", "21010130", "21010135", "21010102" ]:
                return None


            cmd_name = CMD_MAP.get(combined.lower(), "???")
            rest = strip_trailing_zeros(last_words[1:])

            if cmd_name == "???":
                # Show all remaining bytes if unknown
                if rest:
                    return f"{ts} {combined} {cmd_name} {' '.join(rest)}"
                else:
                    return f"{ts} {combined} {cmd_name}"
            else:
                # Known command → only keep non-zero rest
                if rest:
                    return f"{ts} {combined} {cmd_name} {' '.join(rest)}"
                else:
                    return f"{ts} {combined} {cmd_name}"
        return None

    for line in lines:
        line = line.strip()
        # Detect timestamp line
        if re.match(r"^\d\d:\d\d:\d\d\.\d+", line):
            if current_ts and hex_data:
                parsed = process_packet(current_ts, hex_data)
                if parsed:
                    results.append(parsed)
                    print(parsed)  # live output
            # Start new packet
            current_ts = line.split(" IP")[0]
            hex_data = []
        elif line.startswith("0x"):
            parts = line.split(":")[1].strip().split()
            hex_data.extend(parts)

    # Handle last packet
    if current_ts and hex_data:
        parsed = process_packet(current_ts, hex_data)
        if parsed:
            results.append(parsed)
            print(parsed)

    # Save to file
    with open(output_file, "w") as f:
        for row in results:
            f.write(row + "\n")

    print(f"\n✅ Parsed {len(results)} packets (ignored heartbeats). Results saved to {output_file}")


if __name__ == "__main__":
    parse_txyx_file("cmds_output2.txt", "parsed_cmds2.txt")
