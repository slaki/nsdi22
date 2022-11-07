#/bin/python3
import socket
import struct
import time
import traceback
import sys


def pack_int(n):
    """Pack an int to a struct."""

    packer = struct.Struct("i")
    return packer.pack(n)

def pack_bunny(act_id,next_id,duration,positions,speeds):
    """Pack a bunny to a struct.

    Args:
        act_id (int): actual bunny id
        next_id (int): next bunny id
        duration (int): duration of the target in ms
        positions (list of float): target positions for each joint
        speeds (list of float): target velocity for each joint
    """

    packer = struct.Struct("iiQ dddddd dddddd")
    #print(act_id,next_id,duration,positions,speeds)
    data = packer.pack(act_id,next_id,int(duration),
        positions[0],
        positions[1],
        positions[2],
        positions[3],
        positions[4],
        positions[5],
        speeds[0],
        speeds[1],
        speeds[2],
        speeds[3],
        speeds[4],
        speeds[5]
    )
    #print(len(data))
    return data

def to_float(x):
    """Logging float conversion."""

    try:
        return float(x)
    except Exception as e:
        print("BAD FLOAT:",x)
        raise e

def get_traj_from_lines(lines):
    """Parse the csv lines as traj. points.

    Note: The first id is 0.
    """

    data = [ [ to_float(x.strip()) for x in l.strip().split(',')] for l in lines ]

    def get_duration(i):
        """ Calculate the duration and convert it to ms."""

        if i+1<len(data):
            return 1000*(data[i+1][0]-data[i][0])
        else:
            return 2000

    def get_next(i):
        """ Get the next bunny id.
        
            You can achieve a looping traj by changing the definition of this function. It can be useful during debugging.
        """

        return i+1
        #if i+1<len(data):
        #    return i+1
        #else:
        #    return 0

    data = [ (i, get_next(i), get_duration(i),data[i][3:9],data[i][9:15]) for i in range(len(data)) ]
    return data

def send_traj(traj,shifting,control_plane_sock,mod):
    """Send traj points to the bfrt code. 
    
    Args:
        traj: list of the traj. points
        shifting: this number will be added to the id and next_id values
        control_plane_sock: tcp connection to the bfrt script
        mod: modulo used to limit the maximum value of the id and next_id values
    """

    for r in traj:
        control_plane_sock.sendall(pack_int(1))
        control_plane_sock.sendall(pack_bunny((r[0]+shifting) % mod,(r[1]+shifting) % mod,r[2],r[3],r[4]))

def upload_traj_in_reset_mode(client_sock,traj,s,mod):
    """Upload the given traj. in reset mode.
    
    Note: It deletes every traj. points in the switch before uploading this one.
    """

    global g_start
    global g_size
    global g_end
    global g_stop
    global g_in_restart

    # clear everything
    s.sendall(struct.Struct("i").pack(3))
    g_start = 0
    g_end = 0
    g_size = 0

    # set bunny id to 0
    s.sendall(struct.Struct("i").pack(12))
    s.sendall(bunny_id_packer.pack(0))

    # upload new traj
    s.sendall(struct.Struct("i").pack(3))
    send_traj(traj,0,s,mod)
    g_end = len(traj)
    g_size = len(traj)

    # add end stop railway switch
    g_stop = g_end-1
    s.sendall(struct.Struct("i").pack(10))
    s.sendall(bunny_id_packer.pack(g_stop))
    s.sendall(bunny_id_packer.pack(g_stop))

def upload_traj_in_append_mode(traj,s,mod):
    """Upload the given traj. in append mode.
    
    Note: Concatenate this traj to the end of the previous one.
    """

    global g_start
    global g_size
    global g_end
    global g_stop
    global g_in_restart

    send_traj(traj,g_end,s,mod)

    g_end = (g_end+len(traj)) % mod # min(g_end+len(traj), len(traj))
    g_size = min( (g_size+len(traj)), mod )
    
    # remove old stop
    if g_stop>=0:
        s.sendall(struct.Struct("i").pack(11))
        s.sendall(bunny_id_packer.pack(g_stop))
    
    # add new stop
    g_stop = g_end-1
    if g_stop>=0:
        s.sendall(struct.Struct("i").pack(10))
        s.sendall(bunny_id_packer.pack(g_stop))
        s.sendall(bunny_id_packer.pack(g_stop))


def free_unused_bunny_data(s):
    """Delete the unnecessary traj points based on the g_start position and the actual bunny id."""

    global g_start
    global g_size
    global g_end
    global g_stop
    global g_in_restart

    # get actual bunny_id
    s.sendall(struct.Struct("i").pack(9))
    actual_id = bunny_id_packer.unpack(s.recv(4,socket.MSG_WAITALL))[0]

    # free some space
    if g_start<actual_id:
        s.sendall(struct.Struct("i").pack(10))
        s.sendall(bunny_id_packer.pack(g_start))
        s.sendall(bunny_id_packer.pack(actual_id-1))
        g_start = actual_id
    
    elif g_start>actual_id:
        s.sendall(struct.Struct("i").pack(10))
        s.sendall(bunny_id_packer.pack(g_start))
        s.sendall(bunny_id_packer.pack(g_size))

        # remove backlink
        #s.sendall(struct.Struct("i").pack(11))
        #s.sendall(bunny_id_packer.pack(g_size-1))

        g_start = 0
        g_size = g_end


bunny_id_packer = struct.Struct("I")
# these variables maintain a ring data structure
g_start = 0 # firstuploaded bunny id
g_end = 0  # last uploaded bunny id
g_size = 0 # highest uploaded traj id + 1
g_stop = -1 # position if the stop redirection (the last tray point is always loop back to itself in order to stop the robot if there is no new traj. to go along)

g_in_restart = False # not used currently

def handle_client(client_sock):
    """ Handle a client request. """

    global g_start
    global g_size
    global g_end
    global g_stop
    global g_in_restart

    #client_sock.settimeout(5)

    # read command type
    pj_int_packer = struct.Struct("!I")
    buff = client_sock.recv(4,socket.MSG_WAITALL)
    if len(buff)==0:
        print("BYE CLIENT!")
        raise Exception("Client disconnected")
    command_type = pj_int_packer.unpack(buff)[0]
    print(buff)

    # read length
    buff = client_sock.recv(4,socket.MSG_WAITALL)
    data_len = pj_int_packer.unpack(buff)[0]

    # read data
    data = client_sock.recv(data_len,socket.MSG_WAITALL)

    # get traj from data
    data = data.decode()
    lines = [ l.strip() for l in data.split("\n")]
    lines = [ l for l in lines if len(l)>0]
    print(lines[:10])
    traj = get_traj_from_lines(lines[1:])

    # send data
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect(("localhost", 5555))

    if command_type==0: # reset mode
        upload_traj_in_reset_mode(client_sock,traj,s,1000)
    else: # append mode
        free_unused_bunny_data(s)
        upload_traj_in_append_mode(traj,s,1000)

    s.sendall(struct.Struct("i").pack(-1))
    s.close()
    client_sock.sendall(pj_int_packer.pack(1))
    print("start:",g_start,"end:",g_end,"size:",g_size,"stop",g_stop)


server = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind(("localhost",10000))
server.listen()

try:
    while True:
        client_sock, client_addr = server.accept()
        try:
            while True:
                handle_client(client_sock)
        except Exception as e:
            print("ERROR DURING CLIENT HANDING")
            print(e)
            traceback.print_exc(file=sys.stdout)
        try:
            client_sock.close()
        except:
            pass
except Exception as e:
    print(e)

print("START SHUTDOWN")
server.close()
