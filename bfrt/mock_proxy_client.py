import socket
import struct
import sys

# check call
if len(sys.argv)<2:
    print("Usage: python3",sys.argv[0],"<command_type> <fname>")
    exit(-1)

# read csv as bytes
data = open(sys.argv[2], mode='rb').read()

# connect to the proxy
s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
s.connect(("localhost",10000))

# send the data
pj_int_packer = struct.Struct("!I")
s.sendall(pj_int_packer.pack(int(sys.argv[1])))
s.sendall(pj_int_packer.pack(len(data)))
s.sendall(data)

s.close()
