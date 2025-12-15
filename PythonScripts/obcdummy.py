import serial
import time

# ====== CONFIG ======
PORT = "COM3"
BAUD = 9600
TIMEOUT = 0.5

CAM_MSG_LEN = 64
OBC_HEADER = 0x0B      # payload espera 0x0B al inicio
OBC_FOOTER = 0x0C      # payload busca 0x0C como footer (0x0B + 1)
# ====================


def xor_checksum_ascii(s: str) -> int:
    c = 0
    for ch in s:
        c ^= ord(ch) & 0xFF
    return c


def build_obc_frame(inner_cmd: str) -> bytes:
    """
    Frame format expected by payload:
    [0]   = 0x0B
    [1..] = ASCII inner_cmd
    next  = 2 ASCII HEX bytes checksum of inner_cmd (XOR)
    next  = 0x0C
    rest  = 0x00 padding to 64 bytes
    """
    inner_cmd = inner_cmd.strip()
    cs = xor_checksum_ascii(inner_cmd)
    cs_ascii = f"{cs:02X}"  # 2 chars

    payload = bytearray()
    payload.append(OBC_HEADER)
    payload.extend(inner_cmd.encode("ascii"))
    payload.extend(cs_ascii.encode("ascii"))
    payload.append(OBC_FOOTER)

    # pad to 64 bytes
    if len(payload) > CAM_MSG_LEN:
        raise ValueError(f"Frame too long ({len(payload)} bytes). Inner cmd too long.")
    payload.extend(b"\x00" * (CAM_MSG_LEN - len(payload)))
    return bytes(payload)


def parse_payload_response(frame: bytes) -> dict:
    """
    Payload response format (from cam_build_response):
    [0]   = 0xCA
    [1..] = ASCII inner response
    next  = 2 ASCII HEX checksum (XOR of inner response)
    next  = 0xCB
    rest  = padding zeros
    """
    if len(frame) != CAM_MSG_LEN:
        return {"ok": False, "error": f"Bad length {len(frame)}"}

    header = frame[0]
    footer = frame.find(bytes([0xCB]))
    if header != 0xCA or footer == -1:
        return {"ok": False, "error": "Not a valid payload response (missing 0xCA/0xCB)"}

    # inner ends 2 bytes before footer (checksum)
    if footer < 1 + 2:
        return {"ok": False, "error": "Footer too early"}

    inner_end = footer - 2
    inner_bytes = frame[1:inner_end]
    cs_ascii = frame[inner_end:footer].decode("ascii", errors="replace")

    inner = inner_bytes.decode("ascii", errors="replace")
    try:
        cs_rx = int(cs_ascii, 16)
    except ValueError:
        return {"ok": False, "error": f"Bad checksum ASCII: {cs_ascii}", "inner": inner}

    cs_calc = xor_checksum_ascii(inner)
    return {
        "ok": (cs_rx == cs_calc),
        "inner": inner,
        "cs_rx": cs_rx,
        "cs_calc": cs_calc,
        "raw": frame,
    }


def send_and_read(ser: serial.Serial, inner_cmd: str, read_timeout_s: float = 1.5) -> None:
    tx = build_obc_frame(inner_cmd)

    print("[TX] Raw:", tx.hex(" "))             # <-- ADD THIS

    ser.reset_input_buffer()
    ser.write(tx)
    ser.flush()

    t0 = time.time()
    rx = bytearray()
    while len(rx) < CAM_MSG_LEN and (time.time() - t0) < read_timeout_s:
        chunk = ser.read(CAM_MSG_LEN - len(rx))
        if chunk:
            rx.extend(chunk)

    if len(rx) != CAM_MSG_LEN:
        print(f"[RX] Timeout / incomplete frame: got {len(rx)} bytes")
        if rx:
            print("[RX] Raw:", rx.hex(" "))     # (already similar)
        return

    print("[RX] Raw:", bytes(rx).hex(" "))      # <-- ADD THIS
    print("[RX] First16:", bytes(rx[:16]).hex(" "))  # <-- ADD THIS

    resp = parse_payload_response(bytes(rx))

    if not resp["ok"]:
        print("[RX] Invalid/failed response:", resp.get("error", "checksum mismatch"))
        if "inner" in resp:
            print("     inner:", resp["inner"])
        if "cs_rx" in resp:
            print(f"     cs_rx={resp['cs_rx']:02X} cs_calc={resp['cs_calc']:02X}")
    else:
        print("[RX] OK  inner:", resp["inner"])
        print(f"     cs={resp['cs_rx']:02X}")


def main():
    commands = [
        ("STS", "Get status (STS) -> payload responds STS + EE + B0B1B2"),
        ("TIM", "Time request (TIM)"),
        ("CAP", "Capture/save gyro log to Mission Flash (CAP)"),
        ("JPG", "JPG command (stubbed in payload)"),
        ("CMC", "CMC command (stubbed)"),
        ("PDN", "PDN command (stubbed)"),
        ("RST", "RST command (stubbed)"),
        ("OWT", "OWT command (stubbed)"),
        ("CAN", "CAN command (stubbed)"),
        ("NUM", "NUM command (stubbed)"),
        ("IMG", "IMG command (stubbed)"),
        ("CMW", "CMW command (stubbed)"),
        ("CMR", "CMR command (stubbed)"),
        ("CUSTOM", "Send a custom 3-letter cmd (or longer, careful with length)"),
        ("QUIT", "Exit"),
    ]

    with serial.Serial(
        PORT,
        BAUD,
        timeout=TIMEOUT,
        parity=serial.PARITY_NONE,   # <<< CAMBIO CLAVE
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS
    ) as ser:

        print(f"Connected to {PORT} @ {BAUD} bps")
        print("Tip: verify your wiring/logic levels + UART settings match (baud/parity).")
        print()

        while True:
            print("\n=== OBC Console ===")
            for i, (cmd, desc) in enumerate(commands, 1):
                print(f"{i:2d}) {cmd:<6} - {desc}")

            choice = input("\nSelect option: ").strip()
            if not choice.isdigit():
                print("Enter a number.")
                continue

            idx = int(choice) - 1
            if idx < 0 or idx >= len(commands):
                print("Out of range.")
                continue

            cmd = commands[idx][0]
            if cmd == "QUIT":
                break

            if cmd == "CUSTOM":
                inner = input("Enter inner command (ASCII): ").strip()
            else:
                inner = cmd

            try:
                send_and_read(ser, inner)
            except Exception as e:
                print("Error:", e)

    print("Disconnected.")


if __name__ == "__main__":
    main()
