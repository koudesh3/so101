"""SVG: one servo's write packet, byte by byte, drawn like wire-frame-syncwrite.svg.

The sync write carries three bytes per servo (ID and a two-byte value); this is what those
three bytes look like as a packet of their own: goal position 2048 to shoulder_pan alone.

    python3 figures/frame_write_one.py > wire-frame-write.svg
"""
import sys

INK, MID, GREY, FAINT, RULE, BLUE = "#121212", "#555555", "#737373", "#9e9e9e", "#d9d9d9", "#007cd7"

ID, INSTR_WRITE, GOAL_POSITION, VALUE = 0x01, 0x03, 0x2A, 2048
params = [GOAL_POSITION, VALUE & 0xFF, VALUE >> 8]          # little-endian value
length = len(params) + 2                                     # instruction + params + checksum
body = [ID, length, INSTR_WRITE, *params]
checksum = ~sum(body) & 0xFF
frame = [0xFF, 0xFF, *body, checksum]
assert frame == [0xFF, 0xFF, 0x01, 0x05, 0x03, 0x2A, 0x00, 0x08, 0xC4]

# (bytes in the field, label, detail lines, belongs to shoulder_pan's three bytes in the sync write)
FIELDS = [
    (2, "HEADER", ["packet start"], False),
    (1, "ID", ["shoulder_pan"], True),
    (1, "LENGTH", ["5 bytes follow"], False),
    (1, "INSTRUCTION", ["03 write", "02 read"], False),
    (1, "ADDRESS", ["register 42", "Goal_Position"], False),
    (2, "VALUE", ["0x0800 = 2048", "low byte first"], True),
    (1, "CHECKSUM", ["inverted sum", "of bytes 02–07"], False),
]

W, H = 880, 190
CELL, PITCH, SLOT = 30, 33, 124        # each field gets a fixed-width slot so its label fits
TOP = 68

out = []
pattern = (f'<pattern id="d0" width="4" height="4" patternUnits="userSpaceOnUse"><g fill="{BLUE}">'
           + "".join(f'<rect x="{x}" y="{y}" width="1" height="1"/>' for x in (0, 2) for y in (0, 2))
           + "</g></pattern>")
out.append(f'<text x="11" y="20" font-size="12" fill="{INK}">Write: goal position 2048 to shoulder_pan alone</text>')
out.append(f'<text x="11" y="37" font-size="11" fill="{MID}">Host to bus. 9 bytes, 90 µs at 1 Mbaud. '
           f'The servo answers with a 6-byte status packet.</text>')
out.append(f'<text x="11" y="52" font-size="11" fill="{MID}"><tspan fill="{BLUE}">Blue</tspan>: the three bytes shoulder_pan keeps inside the sync write.</text>')

i = 0
for f_i, (n, name, detail, mine) in enumerate(FIELDS):
    slot_x = 11 + SLOT * f_i
    width = CELL + PITCH * (n - 1)
    x0 = slot_x + (SLOT - 4 - width) / 2
    for k in range(n):
        x = x0 + PITCH * k
        if mine:
            out.append(f'<rect x="{x:g}" y="{TOP}" width="{CELL}" height="{CELL + 4}" fill="url(#d0)"/>')
            out.append(f'<rect x="{x + .5:g}" y="{TOP + .5}" width="{CELL - 1}" height="{CELL + 3}" fill="none" stroke="{INK}" stroke-width="1"/>')
            out.append(f'<rect x="{x:g}" y="{TOP}" width="{CELL}" height="3" fill="{BLUE}"/>')
        else:
            out.append(f'<rect x="{x + .5:g}" y="{TOP + .5}" width="{CELL - 1}" height="{CELL + 3}" fill="#ffffff" stroke="{INK}" stroke-width="1"/>')
        out.append(f'<text x="{x + CELL / 2:g}" y="{TOP + 22}" font-size="12" text-anchor="middle" class="m" fill="{INK}">{frame[i]:02X}</text>')
        out.append(f'<text x="{x + CELL / 2:g}" y="{TOP + 46}" font-size="9" text-anchor="middle" class="m" fill="{GREY}">{i:02d}</text>')
        i += 1
    cx = x0 + width / 2
    out.append(f'<path d="M{x0 + 1:g} {TOP + 52}v5H{x0 + width - 1:g}v-5" fill="none" stroke="{BLUE if mine else FAINT}" stroke-width="1"/>')
    out.append(f'<text x="{cx:g}" y="{TOP + 70}" font-size="10" text-anchor="middle" letter-spacing="0.80" fill="{MID}">{name}</text>')
    for row, line in enumerate(detail):
        out.append(f'<text x="{cx:g}" y="{TOP + 86 + 14 * row}" font-size="10" text-anchor="middle" class="m" fill="{GREY}">{line}</text>')

aria = ("One write packet to a single servo, 9 bytes: header FF FF, ID 01 (shoulder_pan), length 05, "
        "instruction 03 (write; 02 is read), address 2A (Goal_Position), value 00 08 (2048, low byte first), "
        f"checksum {checksum:02X}. The ID and value are the three bytes shoulder_pan gets inside the sync write.")
sys.stdout.write(
    f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" height="{H}" font-size="12" role="img" aria-label="{aria}">'
    "<style>text{font-family:-apple-system, BlinkMacSystemFont, 'SF Pro Text', 'Inter', 'Helvetica Neue', Arial, sans-serif}"
    ".m{font-family:'SF Mono', ui-monospace, 'JetBrains Mono', Menlo, Consolas, monospace}</style>\n"
    f"<defs>{pattern}</defs>\n"
    f'<rect width="{W}" height="{H}" fill="#ffffff"/>\n'
    + "\n".join(out) + "\n</svg>\n")
