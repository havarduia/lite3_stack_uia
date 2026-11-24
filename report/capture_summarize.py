"""Summarise a capture: per (src, dst port, size, code) the count and rate, and the 12-byte
simple commands (code, value) in time order. Uses tcpdump -r to read, so no pcap parsing here.

    python3 summarize.py cap.pcap [--commands]
"""
import collections
import re
import struct
import subprocess
import sys

pcap = sys.argv[1]
out = subprocess.run(['tcpdump', '-nn', '-tt', '-x', '-r', pcap], capture_output=True, text=True).stdout

packets = []          # (t, src, dst, payload bytes)
head = re.compile(r'^(\d+\.\d+) .*?IP (\S+) > (\S+): UDP, length (\d+)')
cur = None
for line in out.splitlines():
    m = head.match(line)
    if m:
        if cur:
            packets.append(cur)
        cur = [float(m.group(1)), m.group(2), m.group(3).rstrip(':'), int(m.group(4)), bytearray()]
    elif cur and line.startswith('\t0x'):
        cur[4] += bytes.fromhex(''.join(line.split(':', 1)[1].split()))
if cur:
    packets.append(cur)

t0 = packets[0][0]
span = packets[-1][0] - t0
groups = collections.defaultdict(list)
for t, src, dst, n, raw in packets:
    off = (raw[0] & 0x0F) * 4 + 8       # IP header + UDP header; short frames carry padding after the payload
    payload = bytes(raw[off:off + n])
    code = struct.unpack_from('<I', payload)[0] if n >= 4 else None
    host = src.rsplit('.', 1)[0]
    dport = dst.rsplit('.', 1)[1]
    groups[(host, dst.rsplit('.', 1)[0], dport, n, code)].append((t - t0, payload))

print('span %.2f s, %d packets' % (span, len(packets)))
for (src, dst, dport, n, code), items in sorted(groups.items(), key=lambda kv: -len(kv[1])):
    print('%-15s -> %-15s :%-5s len %-4d code %-12s %6d pkts  %7.1f /s' % (
        src, dst, dport, n, '0x%08X' % code if code is not None else '-', len(items), len(items) / span))

if '--commands' in sys.argv:
    print('\n12-byte packets INTO the controller, in order:')
    for t, src, dst, n, raw in packets:
        if n == 12 and dst.rsplit('.', 1)[1] == '43893':
            off = (raw[0] & 0x0F) * 4 + 8
            code, value, _ = struct.unpack('<IiI', bytes(raw[off:off + 12]))
            print('%8.3f  %-20s 0x%08X  value %d' % (t - t0, src, code, value))
