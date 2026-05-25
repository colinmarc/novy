# /// script
# requires-python = ">=3.11"
# dependencies = ["numpy"]
# ///

"""Decode CAN frames from a sigrok logic analyzer capture.

CAN 2.0A/B at 125kbps on D0 (inverted: idle=LOW in capture means
recessive=HIGH on bus).
"""

import sys
import zipfile
import numpy as np


def load_sr(path):
    with zipfile.ZipFile(path) as zf:
        meta = zf.read("metadata").decode()
        samplerate = 1_000_000
        for line in meta.splitlines():
            if line.startswith("samplerate="):
                val = line.split("=")[1].strip()
                if "MHz" in val:
                    samplerate = int(float(val.replace(" MHz", "")) * 1_000_000)
        chunks = []
        i = 1
        while True:
            try:
                chunks.append(zf.read(f"logic-1-{i}"))
            except KeyError:
                break
            i += 1
    return np.frombuffer(b"".join(chunks), dtype=np.uint8), samplerate


def sample_bits(samples, edges, channel, bit_period, invert=True):
    """Sample NRZ bitstream at bit centers between edges."""
    mask = 1 << channel
    if len(edges) < 2:
        return []

    bits = []
    # Start sampling from half a bit period after first edge.
    first = edges[0]
    last = edges[-1] + bit_period * 2
    last = min(last, len(samples) - 1)

    pos = first + bit_period // 2
    while pos < last:
        raw = bool(samples[int(pos)] & mask)
        bit = (not raw) if invert else raw
        bits.append(1 if bit else 0)
        pos += bit_period

    return bits


def unstuff(bits):
    """Remove CAN bit stuffing. Returns (unstuffed_bits, ok)."""
    out = []
    same_count = 0
    prev = None

    for b in bits:
        if prev is not None and b == prev:
            same_count += 1
        else:
            same_count = 1

        if same_count == 6:
            # Stuff error: 6 consecutive same bits not allowed.
            return out, False

        if same_count == 5:
            # Next bit should be a stuff bit (opposite polarity).
            # We add current bit to output, but the NEXT bit is stuff.
            out.append(b)
            prev = b
            # Skip happens on next iteration — we need to track it.
            same_count = 5  # Flag for next iteration.
            continue

        if prev is not None and same_count == 1 and out:
            # Check if previous was at count=5 (stuff bit to skip).
            pass

        out.append(b)
        prev = b

    return out, True


def decode_can_frame(bits):
    """Try to decode a CAN 2.0A frame from unstuffed bits.

    CAN 2.0A frame (without stuffing):
    SOF(1) + ID(11) + RTR(1) + IDE(1) + R0(1) + DLC(4) +
    DATA(0-64) + CRC(15) + CRC_DEL(1) + ACK(1) + ACK_DEL(1) + EOF(7)
    """
    # Remove stuff bits first.
    unstuffed = []
    same_count = 1
    skip_next = False

    for i, b in enumerate(bits):
        if skip_next:
            skip_next = False
            same_count = 1
            continue

        unstuffed.append(b)

        if i > 0 and b == bits[i - 1] and not skip_next:
            same_count += 1
        else:
            same_count = 1

        if same_count == 5 and i + 1 < len(bits):
            skip_next = True
            same_count = 0

    bits = unstuffed

    if len(bits) < 19:  # Minimum: SOF + ID + RTR + IDE + R0 + DLC
        return None

    pos = 0
    # SOF.
    if bits[pos] != 0:  # SOF is dominant (0).
        return None
    pos += 1

    # 11-bit identifier.
    can_id = 0
    for i in range(11):
        can_id = (can_id << 1) | bits[pos]
        pos += 1

    # RTR bit.
    rtr = bits[pos]
    pos += 1

    # IDE bit (0 = standard frame).
    ide = bits[pos]
    pos += 1

    if ide == 1:
        # Extended frame — 18 more ID bits.
        for i in range(18):
            can_id = (can_id << 1) | bits[pos]
            pos += 1
        # RTR for extended.
        rtr = bits[pos]
        pos += 1
        # R1, R0.
        pos += 2
    else:
        # R0.
        pos += 1

    # DLC (4 bits).
    if pos + 4 > len(bits):
        return None
    dlc = 0
    for i in range(4):
        dlc = (dlc << 1) | bits[pos]
        pos += 1
    dlc = min(dlc, 8)

    # Data bytes.
    data = []
    for byte_i in range(dlc):
        if pos + 8 > len(bits):
            return {"id": can_id, "ide": ide, "rtr": rtr, "dlc": dlc,
                    "data": data, "complete": False}
        val = 0
        for i in range(8):
            val = (val << 1) | bits[pos]
            pos += 1
        data.append(val)

    # CRC (15 bits).
    crc_bits = []
    if pos + 15 <= len(bits):
        for i in range(15):
            crc_bits.append(bits[pos])
            pos += 1

    return {"id": can_id, "ide": ide, "rtr": rtr, "dlc": dlc,
            "data": data, "complete": True,
            "crc_bits": crc_bits, "total_bits": pos}


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "cap2.sr"
    samples, sr = load_sr(path)
    bit_period = sr // 125000  # 8 samples at 1MHz

    print(f"Loaded {len(samples)} samples at {sr/1e6:.1f} MHz")
    print(f"CAN bit period: {bit_period} samples ({bit_period}µs)\n")

    # Find edges on D0.
    mask = 1
    raw_bits = (samples & mask).astype(bool)
    edge_indices = np.where(raw_bits[1:] != raw_bits[:-1])[0] + 1

    # Group into bursts (packets). Gap threshold must be between
    # max intra-frame recessive (5 bits = 40us) and interframe
    # space (EOF+IFS = 10 bits = 80us). Scale by sample rate.
    gap_samples = int(50 * (sr / 1_000_000))  # 50us
    splits = np.where(np.diff(edge_indices) > gap_samples)[0] + 1
    bursts = np.split(edge_indices, splits)
    bursts = [b for b in bursts if len(b) >= 4]

    print(f"Found {len(bursts)} bursts\n")
    print(f"{'#':>4} {'Time':>9} {'ID':>6} {'DLC':>4} {'Data':>30} {'Note'}")
    print("-" * 70)

    # Try both polarities.
    for invert in [True, False]:
        decoded = 0
        for bi, burst in enumerate(bursts[:50]):
            t_ms = burst[0] / (sr / 1000)
            bits = sample_bits(samples, burst, 0, bit_period, invert=invert)
            frame = decode_can_frame(bits)

            if frame and frame.get("complete") and len(frame["data"]) == frame["dlc"]:
                decoded += 1

        if decoded > 5:
            polarity = "inverted" if invert else "normal"
            print(f"Using {polarity} polarity ({decoded} valid frames)\n")

            for bi, burst in enumerate(bursts):
                t_ms = burst[0] / (sr / 1000)
                dur_ms = (burst[-1] - burst[0]) / (sr / 1000)
                bits = sample_bits(samples, burst, 0, bit_period, invert=invert)
                frame = decode_can_frame(bits)

                if frame:
                    id_hex = f"0x{frame['id']:03X}" if not frame['ide'] else f"0x{frame['id']:08X}"
                    data_hex = " ".join(f"{b:02X}" for b in frame["data"])
                    note = ""
                    if not frame.get("complete"):
                        note = "INCOMPLETE"
                    if frame["rtr"]:
                        note = "RTR"
                    print(f"{bi:4d} {t_ms:8.1f}ms {id_hex:>6} "
                          f"{frame['dlc']:4d} {data_hex:>30} {note}")
                else:
                    n_bits = len(bits)
                    print(f"{bi:4d} {t_ms:8.1f}ms {'???':>6} "
                          f"{'':4} {'(decode failed)':>30} "
                          f"bits={n_bits} dur={dur_ms:.2f}ms")
            break
    else:
        print("Neither polarity produced valid CAN frames.")
        print("\nRaw bits from first burst (both polarities):")
        if bursts:
            bits_inv = sample_bits(samples, bursts[0], 0, bit_period, True)
            bits_nor = sample_bits(samples, bursts[0], 0, bit_period, False)
            print(f"  Inverted: {''.join(str(b) for b in bits_inv[:60])}")
            print(f"  Normal:   {''.join(str(b) for b in bits_nor[:60])}")


if __name__ == "__main__":
    main()
