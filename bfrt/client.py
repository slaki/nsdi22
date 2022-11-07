#/bin/python3
import socket
import struct
import time

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(("localhost", 5555))

# ***** HELPER FUNCTIONS *****

def pack_int(n):
    """Pack an int to a struct."""

    packer = struct.Struct("i")
    return packer.pack(n)

def pack_bunny(robot_id,act_id,next_id,duration,positions,speeds):
    """Pack a bunny to a struct.

    Args:
        robot_id (int): robot id
        act_id (int): actual bunny id
        next_id (int): next bunny id
        duration (int): duration of the target in ms
        positions (list of float): target positions for each joint
        speeds (list of float): target velocity for each joint
    """

    packer = struct.Struct("iiiQ dddddd dddddd")
    #print(act_id,next_id,duration,positions,speeds)
    data = packer.pack(robot_id,act_id,next_id,int(duration),
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

def pack_function_add_command(c,speed,mask,value):
    """Pack a 'add function entry' command to a struct.

    Args:
        c (str): function id 
        speed (int): match key value
        mask (int): mask for the ternary match
        value (int): function value 

    Valid c values:
        'A': function on the actual speed
        'T': function on the target speed
        'D': function on the position difference
    """

    packer = struct.Struct("cQQQ")
    return packer.pack(c.encode(),
        min(speed,18446744073709551615),
        min(mask,18446744073709551615),
        min(value,18446744073709551615))

# ***** TRAJ HELPER FUNCTIONS *****

def get_traj_from_file(fname):
    """Load a traj from a file.
    
    Note: the first id is 0.
    """

    lines = [ l.strip() for l in open(fname)]
    data = [ [ float(x.strip()) for x in l.split(',')] for l in lines[1:] ]

    def get_duration(i):
        if i+1<len(data):
            return 1000*(data[i+1][0]-data[i][0])
        else:
            return 2000

    def get_next(i):
        if i+1<len(data):
            return i+1
        else:
            return 0

    data = [ (i, get_next(i), get_duration(i),data[i][3:9],data[i][9:15]) for i in range(len(data)) ]
    return data

class TrajProvider:
    """ This class helps to upload traj. episodes based on the durations and sleep command.

    Note: only for testing.
    """

    def __init__(self,fname,start_id=0):
        self.traj = get_traj_from_file(fname)
        self.start_id = 0
        self.provided = []
        
    def get_batch(self,duration):
        "returns the next episode"
        pack = []
        d = 0.0
        while d<duration:
            pack.append(self.traj[self.start_id])
            self.provided.append((self.start_id,self.traj[self.start_id][2]))
            d+=self.traj[self.start_id][2]
            self.start_id = self.traj[self.start_id][1]
        return pack
    
    def get_free(self,duration):
        "returns the bunny ids older than the given duration, and removes it from the 'provided' list"
        d = 0.0
        i = len(self.provided)-1
        while d<duration:
            if i==-1:
                return []
            
            d+=self.provided[i][1]
            i-=1
        
        result = [ x[0] for x in self.provided ][0:i]
        self.provided = self.provided[i:]
        return result
    
    def get_free_ranges(self,duration):
        bs = self.get_free(duration)
        if len(bs)==0:
            return []
        result = []
        first, last = -1,-1
        for n in bs:
            if first == -1:
                first = n
                last = n
            elif n == last+1:
                last = n
            else:
                result.append((first,last))
                first = n
                last = n
        result.append((first,last))
        return result

# ***** RULE GENERATOR FUNCTIONS *****

import math
import numpy as np

INT_MAX = 2147483647

def bnot(b):
    """Logical 'not' for a single character input. """

    if b=='0':
        return "1"
    else:
        return '0'
        
def my_not(bs):
    """Logical 'not' for 0-1 strings. """

    return "".join([ bnot(b) for b in bs])

def unsigned_to_neg_signed(n):
    """Parse the binary representation of an unsigned integer as a signed integer.
    
    Note: it is like casting in C or C++.
    """

    bs = bin(n-1)[2:]
    bs = my_not(bs)
    return -1*int(bs,2)

def get_variations(n):
    """Return every possible n long string built from 0 and 1 characters."""

    bs = [ bin(i)[2:] for i in range(2**n) ]
    bs = [ '0'*(n-len(b))+b for b in bs ]
    return [ b for b in bs ]
    
def get_prefixes(precision):
    """Return the possible prefixes of the binary representation if 64-bit long integers containing the 'precision' meaningful bits.
    
    Meaningful: not 0 or not 1 depending on the signum of the represented value.
    """

    vs = get_variations(precision)
    # positive part
    pos = [ i*'0'+"1"+p for i in range(1,64-precision-1) for p in vs ]
    # negative part
    neg = [ i*'1'+"0"+p for i in range(1,64-precision-1) for p in vs ]
    return pos+neg

def get_min_max_by_prefix(prefix):
    """Get the possible minimum and maximum value of a 64-bit long integer based on its prefix."""

    if prefix[0]=='0':
        return list(sorted([
            int(prefix+'0'*(64-len(prefix)),2),
            int(prefix+'1'*(64-len(prefix)),2)        
        ]))
    else:
        return list(sorted([
            unsigned_to_neg_signed(int(prefix+'0'*(64-len(prefix)),2)),
            unsigned_to_neg_signed(int(prefix+'1'*(64-len(prefix)),2))      
        ]))
        
def get_ternary_entries(func,precision):
    """Generate the ternary entries for the provided function with the provided precision.
    
    Args:
        func (function: list of ints --> int): the function to be approximated
        precision (int): the number of important bits used during value measurement
     
    Return format: [ (case, mask, value, debug...) ] 
    """

    ps = get_prefixes(precision)
    # return [ (case, mask, value, debug...) ]
    return [ (p+'0'*(64-len(p)),
              '1'*len(p)+'0'*(64-len(p)),
              func(get_min_max_by_prefix(p)),
              get_min_max_by_prefix(p)) 
                    for p in ps ]

def get_multiplicator(c):
    """Return a 'multiply' function that can be used as an input for the get_ternary_entries function.
    
    Args:
        c (float): constant used by the returned 'multiply' function
    """

    def mult(xs):
        r = int(min(xs)*c)
        r = np.binary_repr(r,width=64)
        return int(r,2)
    return mult


# ***** HANDLER FUNCTIONS *****

def handle_exit(code):
    s.sendall(pack_int(code))

def handle_add():
    robot_id = int(input("\trobot_id : "))
    bunny_id = int(input("\tbunny_id : "))
    next_id  = int(input("\tnext_id  : "))
    duration = int(input("\tduration : "))
    print("\tpositions:")
    positions = [ float(input("\t\tjoint "+str(i+1)+": ")) for i in range(6)]
    print("\tspeeds:")
    speeds = [ float(input("\t\tjoint "+str(i+1)+": ")) for i in range(6)]
    s.sendall(pack_int(1))
    s.sendall(pack_bunny(robot_id,bunny_id,next_id,duration,positions,speeds))

def handle_delete():
    robot_id = int(input("\trobot_id : "))
    first = int(input("\tfirst : "))
    last = int(input( "\tlast  : "))
    s.sendall(pack_int(2))
    s.sendall(pack_int(robot_id))
    s.sendall(pack_int(first))
    s.sendall(pack_int(last))

def handle_clear():
    s.sendall(pack_int(3))

def handle_load():
    try:
        robot_id = int(input("\trobot id : "))
        fname = input("\tinput file : ")
        shifting = int(input( "\tshifting   : "))
        data = get_traj_from_file(fname)
        for r in data:
            s.sendall(pack_int(1))
            s.sendall(pack_bunny(robot_id,r[0]+shifting,r[1]+shifting,r[2],r[3],r[4]))
    except Exception as e:
        print(e)

def handle_provider():
    try:
        robot_id = int(input("\trobot id : "))
        fname = input("\tinput file : ")
        shifting = int(input( "\tshifting   : "))
        episode_count = int(input( "\tnum. of. ep.   : "))
        tp = TrajProvider(fname)
        data = tp.get_batch(5000)
        buff = sum([r[2] for r in data])
        for r in data:
            s.sendall(pack_int(1))
            s.sendall(pack_bunny(robot_id,r[0]+shifting,r[1]+shifting,r[2],r[3],r[4]))
        time.sleep(3)
        buff = buff - 3000
        for i in range(episode_count):
            # delete old bunny records
            rs = tp.get_free_ranges(int(buff*1.1))
            print("\t\tRemove\t",rs)
            for r in rs:
                s.sendall(pack_int(2))
                s.sendall(pack_int(r[0]))
                s.sendall(pack_int(r[1]))

            # upload new episode
            data = tp.get_batch(1000)
            buff = buff + sum([r[2] for r in data])
            print("\tUpload:\t",[data[0][0],data[-1][0]])
            for r in data:
                s.sendall(pack_int(1))
                s.sendall(pack_bunny(robot_id,r[0]+shifting,r[1]+shifting,r[2],r[3],r[4]))
            time.sleep(1)
            buff = buff - 1000


    except Exception as e:
        print(e.with_traceback())


def handle_dump():
    s.sendall(pack_int(5))

def handle_function_clear():
    s.sendall(pack_int(7))

def handle_weighting():
    c_act =  float(input("\tweight of ACTUAL speed> "))
    c_tar =  float(input("\tweight of TARGET speed> "))
    c_diff = float(input("\tweight of DIFF   speed> "))

    s.sendall(pack_int(7))

    # diff
    es = get_ternary_entries(get_multiplicator(c_diff),4)
    for e in es:
        s.sendall(pack_int(8))
        s.sendall(pack_function_add_command("D",int(e[0],2),int(e[1],2),e[2]))

    # actual
    es = get_ternary_entries(get_multiplicator(c_act),4)
    for e in es:
        s.sendall(pack_int(8))
        s.sendall(pack_function_add_command("A",int(e[0],2),int(e[1],2),e[2]))

    # target
    es = get_ternary_entries(get_multiplicator(c_tar),4)
    for e in es:
        s.sendall(pack_int(8))
        s.sendall(pack_function_add_command("T",int(e[0],2),int(e[1],2),e[2]))

def handle_actual_bunny_id():
    robot_id = int(input("\trobot id> ")) 
    s.sendall(pack_int(9))
    s.sendall(pack_int(robot_id))
    buff = s.recv(4,socket.MSG_WAITALL)
    act_id = struct.Struct("I").unpack(buff)[0]
    print("\tActual bunny id:",act_id)

def handle_set_actual_bunny_id():
    robot_id = int(input("\trobot id> ")) 
    bid = int(input("\tnew id> "))
    s.sendall(pack_int(12))
    s.sendall(pack_int(robot_id))
    s.sendall(pack_int(bid))

bunny_id_packer = struct.Struct("I")

def handle_set_railway():
    robot_id = int(input("\trobot id> ")) 
    from_id = int(input("\tfrom> "))
    to_id   = int(input("\tto> "))
    s.sendall(pack_int(10))
    s.sendall(pack_int(robot_id))
    s.sendall(bunny_id_packer.pack(from_id))
    s.sendall(bunny_id_packer.pack(to_id))

def handle_unset_railway():
    robot_id = int(input("\trobot id> ")) 
    from_id = int(input("\tfrom> "))
    s.sendall(pack_int(11))
    s.sendall(pack_int(robot_id))
    s.sendall(bunny_id_packer.pack(from_id))

# ***** MAIN *****

while True:
    print("Select a command:\n\t"+
        "1)  add bunny\n\t"+
        "2)  delete range\n\t"+
        "3)  clear bunny data\n\t"+
        "4)  load traj from file\n\t"+
        "5)  dump on server\n\t"+
        "6)  use episode provider\n\t"+
        "\n\t"+
        "7)  clear weight functions\n\t"+
        "8)  upload weight functions\n\t"+
        "\n\t"+
        "9)  get actual bunny id\n\t"+
        "10) set railway switch\n\t"+
        "11) unset railway switch\n\t"+
        "12) set actual bunny id\n\t"+
        "\n\t"+
        "-1) exit\n\t"+
        "-2) stop server\n\n")
    cmd = int(input("command number> "))
    if cmd==-1:
        handle_exit(-1)
        break
    elif cmd==-2:
        handle_exit(-2)
        break
    elif cmd==1:
        handle_add()
    elif cmd==2:
        handle_delete()
    elif cmd==3:
        handle_clear()
    elif cmd==4:
        handle_load()
    elif cmd==5:
        handle_dump()
    elif cmd==6:
        handle_provider()
    elif cmd==7:
        handle_function_clear()
    elif cmd==8:
        handle_weighting()
    elif cmd==9:
        handle_actual_bunny_id()
    elif cmd==10:
        handle_set_railway()
    elif cmd==11:
        handle_unset_railway()
    elif cmd==12:
        handle_set_actual_bunny_id()
    else:
        print("ERR: Invalid command number: "+str(cmd))

s.close()
