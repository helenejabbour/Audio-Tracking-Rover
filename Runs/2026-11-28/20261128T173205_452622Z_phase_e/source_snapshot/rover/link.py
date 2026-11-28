"""Open the ESP32 V2 serial link and prove it carries valid frames before a run starts.

Two failure modes seen on 28 Nov (16 of 93 captures lost):
- Opening the port resets the ESP32; bytes received during the reset sit in its 32-byte line
  buffer, so the first command is concatenated with garbage and rejected ("# invalid command").
  A leading newline terminates that partial line first.
- The CP210x link sometimes comes up at a wrong effective baud: lines arrive at the pair rate
  but are noise, and every R2 fails its CRC. Closing and reopening the port fixes it, so the
  handshake retries before giving up.
A run that never sees a valid frame now fails within seconds with a reason, instead of
recording garbage for its whole duration.
"""
import time

from .sensing import parse_v2


class LinkError(RuntimeError):
    pass


def valid_frame(text):
    """True for a C2 configuration line or an R2 line with a correct CRC."""
    return (text.startswith('C2,') and 'fs=' in text and 'N=' in text) or \
        (text.startswith('R2,') and parse_v2(text) is not None)


def open_link(port, commands, baud=115200, attempts=3, handshake_s=4., boot_s=2.,
              serial_factory=None, clock=time.monotonic, sleep=time.sleep):
    """Return (ser, lines): an open port that has delivered a valid frame, and the
    [(monotonic, text)] lines read during the handshake (the caller logs and parses them).

    commands: newline-separated firmware commands; '?' is appended so the firmware answers
    with its C2 line. Raises LinkError after `attempts` failed opens.
    """
    if serial_factory is None:
        import serial
        serial_factory = serial.Serial
    payload = ('\n' + commands.strip('\n') + '\n?\n').encode('ascii')
    reasons = []
    for attempt in range(1, attempts+1):
        try:
            ser = serial_factory(port, baud, timeout=.05, write_timeout=.5)
        except Exception as exc:   # SerialException / OSError: busy, unplugged, re-enumerating
            reasons.append(f'attempt {attempt}: open failed: {exc}')
            sleep(1.)
            continue
        sleep(boot_s); ser.reset_input_buffer(); ser.write(payload)
        lines, buffer, start, good = [], b'', clock(), False
        while clock()-start < handshake_s and not good:
            buffer += ser.read(ser.in_waiting or 1)
            while b'\n' in buffer:
                raw, buffer = buffer.split(b'\n', 1)
                text = raw.decode('ascii', errors='replace').strip()
                if text:
                    lines.append((clock(), text))
                    good = good or valid_frame(text)
        if good:
            return ser, lines
        garbled = sum(1 for _, t in lines if not t.isprintable() or '�' in t)
        reasons.append(f'attempt {attempt}: {len(lines)} lines in {handshake_s:.0f} s, none valid'
                       + (f' ({garbled} garbled: wrong effective baud)' if garbled else ''))
        ser.close()
    raise LinkError(f'{port}: no valid C2/R2 frame after {attempts} attempts; ' + '; '.join(reasons)
                    + '. Check the ESP32 USB cable/port and that ModemManager is disabled.')
