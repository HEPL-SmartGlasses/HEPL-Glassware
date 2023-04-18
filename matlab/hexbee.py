import struct

f1 = 16.96
bytes1 = bytearray(struct.pack("f", f1))

f2 = 70.48
bytes2 = bytearray(struct.pack("f", f2))

status = 0
bytes3 = bytearray(struct.pack("i", status))

# print the raw bytes
print("".join([f"{b:02x}" for b in bytes1]), end="")
print("".join([f"{b:02x}" for b in bytes2]), end="")
print("".join([f"{b:02x}" for b in bytes3]))
