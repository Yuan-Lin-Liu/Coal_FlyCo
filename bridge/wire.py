"""Length-delimited JSON metadata and NumPy arrays over a local TCP socket."""
import json
import math
import struct

import numpy as np

MAX_BYTES = 32 * 1024 * 1024


def read_exact(sock, size):
    chunks = bytearray(size)
    view = memoryview(chunks)
    while view:
        count = sock.recv_into(view)
        if not count:
            raise ConnectionError('Simulator connection closed')
        view = view[count:]
    return chunks


def send_packet(sock, metadata, arrays=None):
    arrays = {k: np.ascontiguousarray(v) for k, v in (arrays or {}).items()}
    header = dict(metadata, arrays={k: {'dtype': v.dtype.str, 'shape': v.shape}
                                   for k, v in arrays.items()})
    data = json.dumps(header, allow_nan=False, separators=(',', ':')).encode()
    size = 4 + len(data) + sum(v.nbytes for v in arrays.values())
    if size > MAX_BYTES:
        raise ValueError('Packet exceeds size limit')
    sock.sendall(struct.pack('!II', size, len(data)) + data)
    for value in arrays.values():
        if value.nbytes:
            sock.sendall(memoryview(value).cast('B'))


def recv_packet(sock):
    size, header_size = struct.unpack('!II', read_exact(sock, 8))
    if not 4 <= size <= MAX_BYTES or not 0 < header_size <= min(size - 4, 65536):
        raise ValueError('Invalid packet length')
    metadata = json.loads(read_exact(sock, header_size))
    specs = metadata.pop('arrays', {})
    remaining = size - 4 - header_size
    arrays = {}
    for key, spec in specs.items():
        dtype = np.dtype(spec['dtype'])
        shape = spec['shape']
        if dtype.kind not in 'fiu' or not shape or len(shape) > 3 or any(
                not isinstance(n, int) or n < 0 for n in shape):
            raise ValueError('Unsupported array specification')
        count = math.prod(shape)
        length = count * dtype.itemsize
        if length > remaining:
            raise ValueError('Array exceeds packet length')
        arrays[key] = np.frombuffer(read_exact(sock, length), dtype=dtype).reshape(shape)
        remaining -= length
    if remaining:
        raise ValueError('Trailing bytes in packet')
    return metadata, arrays
