import serial
import time
from dataclasses import dataclass

# ====== CONFIG ======
PORT = "COM3"
BAUD = 9600
TIMEOUT = 0.1

CAM_MSG_LEN = 64
OBC_HEADER = 0x0B      # OBC -> Payload
CAM_HEADER = 0xCA      # Payload -> OBC

STS_PERIOD_S = 0.25     # cada cuánto mandar STS durante preflight
READ_TIMEOUT_S = 1.5    # timeout para recibir 64 bytes

INTER_CMD_DELAY_S = 0.4  # delay entre comandos en la secuencia
RETRIES_PER_CMD = 3       # reintentos por comando si falla parse/checksum

READ_TIMEOUT_CAP_S = 60.0   # o 90.0 si quieres más margen

# ====================
def read_frame_sync(ser: serial.Serial, header: int, timeout_s: float) -> bytes | None:
    t0 = time.time()
    while (time.time() - t0) < timeout_s:
        b = ser.read(1)
        if not b or b[0] != header:
            continue
        rest = ser.read(CAM_MSG_LEN - 1)
        if len(rest) == CAM_MSG_LEN - 1:
            return bytes([header]) + rest
    return None

def cam_checksum_xor(s: str) -> int:
    c = 0
    for ch in s:
        c ^= ord(ch) & 0xFF
    return c & 0xFF


def cam_generate_cmd(header: int, inner: str) -> bytes:
    inner = inner.strip()
    cs = cam_checksum_xor(inner)
    cs_ascii = f"{cs:02X}"

    out = bytearray(CAM_MSG_LEN)
    out[0] = header

    inner_b = inner.encode("ascii")
    if 1 + len(inner_b) + 2 + 1 > CAM_MSG_LEN:
        raise ValueError("Inner demasiado largo para frame de 64 bytes.")

    out[1:1 + len(inner_b)] = inner_b
    out[1 + len(inner_b):1 + len(inner_b) + 2] = cs_ascii.encode("ascii")
    out[1 + len(inner_b) + 2] = (header + 1) & 0xFF  # footer
    return bytes(out)


@dataclass
class ParsedFrame:
    ok: bool
    error: str = ""
    inner: str = ""
    cs_rx: int = 0
    cs_calc: int = 0
    raw: bytes = b""


def parse_cam_frame(expected_header: int, frame: bytes) -> ParsedFrame:
    if len(frame) != CAM_MSG_LEN:
        return ParsedFrame(False, f"Bad length: {len(frame)}", raw=frame)

    if frame[0] != expected_header:
        return ParsedFrame(False, f"Bad header 0x{frame[0]:02X} (expected 0x{expected_header:02X})", raw=frame)

    footer_byte = (expected_header + 1) & 0xFF
    footer_idx = frame.find(bytes([footer_byte]), 1)
    if footer_idx == -1:
        return ParsedFrame(False, f"Footer 0x{footer_byte:02X} not found", raw=frame)

    if footer_idx < 1 + 2:
        return ParsedFrame(False, "Footer too early", raw=frame)

    cs_ascii = frame[footer_idx - 2:footer_idx].decode("ascii", errors="replace")
    try:
        cs_rx = int(cs_ascii, 16)
    except ValueError:
        return ParsedFrame(False, f"Bad checksum ASCII: {cs_ascii!r}", raw=frame)

    inner_bytes = frame[1:footer_idx - 2]
    inner = inner_bytes.decode("ascii", errors="replace")

    cs_calc = cam_checksum_xor(inner)
    if cs_rx != cs_calc:
        return ParsedFrame(False, "Checksum mismatch", inner=inner, cs_rx=cs_rx, cs_calc=cs_calc, raw=frame)

    return ParsedFrame(True, inner=inner, cs_rx=cs_rx, cs_calc=cs_calc, raw=frame)

def send_cmd(ser: serial.Serial, inner_cmd: str, read_timeout_s: float = READ_TIMEOUT_S) -> ParsedFrame:
    tx = cam_generate_cmd(OBC_HEADER, inner_cmd)

    # ❌ NO hagas esto:
    # ser.reset_input_buffer()

    ser.write(tx)
    ser.flush()

    rx = read_frame_sync(ser, CAM_HEADER, timeout_s=read_timeout_s)
    if rx is None:
        return ParsedFrame(False, f"RX timeout/incomplete (<{CAM_MSG_LEN} bytes)")

    parsed = parse_cam_frame(CAM_HEADER, rx)
    if not parsed.ok:
        parsed.raw = rx
    return parsed

def is_sts_ok(inner: str) -> bool:
    """
    Tu payload arma: "STS%02X%c%c%c"
    Queremos EE == 00 para 'todo bien'.
    """
    if not inner.startswith("STS") or len(inner) < 5:
        return False
    ee_hex = inner[3:5]
    try:
        ee = int(ee_hex, 16)
    except ValueError:
        return False
    return ee == 0x00


def preflight_wait_sts_ok(ser: serial.Serial) -> str:
    """
    Manda STS continuamente hasta ver STS00...
    Devuelve el último inner válido que confirmó OK.
    """
    print("\n[PRE] Polling STS hasta STS00...")
    last_good = ""
    while True:
        parsed = send_cmd(ser, "STS")
        if parsed.ok:
            last_good = parsed.inner
            if is_sts_ok(parsed.inner):
                print(f"[PRE] OK: {parsed.inner!r}")
                return parsed.inner
            else:
                print(f"[PRE] NOT OK: {parsed.inner!r}")
        else:
            print(f"[PRE] STS failed: {parsed.error}")
        time.sleep(STS_PERIOD_S)


def run_sequence(ser: serial.Serial, sequence: list[str]) -> None:
    print("\n[SEQ] Iniciando rutina:", sequence)

    for cmd in sequence:
        success = False
        last_error = ""

        for attempt in range(1, RETRIES_PER_CMD + 1):
            rt = READ_TIMEOUT_CAP_S if cmd == "CAP" else READ_TIMEOUT_S
            parsed = send_cmd(ser, cmd, read_timeout_s=rt)

            if parsed.ok:
                print(f"[SEQ] {cmd} -> {parsed.inner!r} (cs=0x{parsed.cs_rx:02X})")

                # Caso lógico de error (ej: CAP01)
                if cmd == "CAP" and not parsed.inner.endswith("00"):
                    print(f"[SEQ] WARN: {cmd} respondió error lógico ({parsed.inner})")
                success = True
                break
            else:
                last_error = parsed.error
                print(
                    f"[SEQ] {cmd} attempt {attempt}/{RETRIES_PER_CMD} FAIL: {parsed.error}"
                )
                time.sleep(0.1)

        if not success:
            print(
                f"[SEQ] ERROR: {cmd} falló tras {RETRIES_PER_CMD} intentos "
                f"(último error: {last_error})"
            )
            print("[SEQ] Continuando con el siguiente comando...\n")

        time.sleep(INTER_CMD_DELAY_S)

    print("[SEQ] Rutina terminada (con o sin errores).")


def ask_repeat_or_quit() -> str:
    while True:
        ans = input("\n¿Repetir rutina? (r=repeat / q=quit): ").strip().lower()
        if ans in ("r", "q"):
            return ans
        
def drain_rx(ser: serial.Serial):
    while ser.in_waiting:
        ser.read(ser.in_waiting)

def main():
    # Ajusta el orden EXACTO que quieras después de que STS00 aparezca
    routine = ["TIM", "CAP", "JPG", "CMC", "PDN", "RST", "OWT", "CAN", "NUM", "IMG", "CMW", "CMR"]

    with serial.Serial(
        PORT,
        BAUD,
        timeout=TIMEOUT,
        parity=serial.PARITY_NONE,   # ST-LINK típico 8N1
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
    ) as ser:
        time.sleep(0.2)
        drain_rx(ser)
        print(f"Connected to {PORT} @ {BAUD} (8N1)")

        while True:
            # 1) STS continuo hasta STS00
            preflight_wait_sts_ok(ser)

            # 2) Secuencia de comandos
            run_sequence(ser, routine)

            # 3) Repetir o salir
            ans = ask_repeat_or_quit()
            if ans == "q":
                break

    print("Disconnected.")


if __name__ == "__main__":
    main()
