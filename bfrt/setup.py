
import socket
import math
import struct
import time

INT_MAX = 2147483647

p4 = bfrt.ur.pipe

# ***** HELPER FUNCTIONS *****

def recv_all(sock,length):
    """Receive the given amount of bytes from the provided TCP socket."""

    global socket
    return sock.recv(length,socket.MSG_WAITALL)


def unpack_bunny_add_command(buff):
    """Unpack the trajectory point (bunny) information from a struct.
    
       The fields of the struct is the following:
       - robot id
       - actual bunny id
       - next bunny id
       - duration of the target in ms
       - target positions for each joint
       - target velocity for each joint
    """

    # actual,next,duration,6 positions,6 speeds
    global struct
    packer = struct.Struct("iiiQ dddddd dddddd")
    data = packer.unpack(buff)
    return data[0],data[1],data[2],data[3],list(data[4:10]),list(data[10:16])

def unpack_int(buff):
    """Unpack a single integer from a struct."""

    packer = struct.Struct("i")
    return packer.unpack(buff)[0]

def unpack_function_add_command(buff):
    """Unpack an 'add function entry' command from a dtruct.
    
    The fields of the struct:
    - function id 
    - key
    - mask
    - function value 
    """

    packer = struct.Struct("cQQQ")
    c,speed,mask,value = packer.unpack(buff)
    c = c.decode()
    return c,speed,mask,value

# ***** CONTROL PLANE FUNCTIONS *****

def run_server():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(("localhost", 5555))
    s.listen(1)
    client_socket, client_addr = s.accept()
    print("Client connected: "+str(client_addr))
    client_socket.sendall("HELLO CLIENT\n".encode())
    client_socket.close()
    s.close()

def dump_bunny_tables():
    """Print the content of some important tables."""

    global p4
    p4.SwitchIngress.bunny.dump()
    p4.SwitchEgress.bunny_e.dump()
    p4.SwitchIngress.railway_switch.dump()


def clear_bunny_data(verbose=True, batching=True):
    """Delete the content of the bunny, bunny_e and railway_switch table."""

    global p4
    global bfrt

    def table_clear(table, verbose=False, batching=False):
        """ Delete the content of the given table. """

        try:
            entries = table.get(regex=True, print_ents=False)
            for r in entries:
                r.remove()
        except Exception as e:
            print(e)

    # cleare Ingress and Egress bunny table
    table_clear(p4.SwitchIngress.bunny)
    table_clear(p4.SwitchEgress.bunny_e)
    table_clear(p4.SwitchIngress.railway_switch)
    print("OK: bunny data cleared")

def add_new_bunny(robot_id,bunny_id,next_id,duration,positions,speeds):
    """ Add a new trajectory point.
    
    Args:
        robot_id: id of the robot which the traj. point belongs
        bunny_id (int): trajectory point id
        next_id (int): id of the next traj. point
        duration (int): time until the next traj. point
        positions (list of floats): positions for each joint
        speeds (list of floats): velocity values for each joint 
    """

    global p4
    global bfrt
    global INT_MAX
    global math

    def msec_to_int(m):
        """Convert the duration to the proper int representation."""
        
        return m*1000000/65536

    def double_to_dec(db):
        """Convert the speed and position values to the proper int representation."""

        return  int(INT_MAX/(16*4*math.pi) * db)

    def add_ingress_part():
        for i in range(6):
            p4.SwitchIngress.bunny.add_with_set_target(
                robot_id=robot_id,
                actual_bunny=bunny_id,
                jointid=i,
                next_id=next_id,
                duration=msec_to_int(duration))

    def add_egress_part():
        for i in range(6):
            p4.SwitchEgress.bunny_e.add_with_set_target_e(
                robot_id=robot_id,
                actual_bunny_id=bunny_id,
                jointid=i,
                tpos=double_to_dec(positions[i]),
                tspeed=double_to_dec(speeds[i])) 
    
    try:
        add_ingress_part()
        add_egress_part()
    except Exception as e:
        print("ERROR DURING BUNNY ADD")
        print(e)

def clear_function_data():
    """Delete the content of the function tables."""

    global p4
    
    def table_clear(table, verbose=False, batching=False):
        """ Delete the content of the given table. """

        try:
            entries = table.get(regex=True, print_ents=False)
            for r in entries:
                r.remove()
        except Exception as e:
            print(e)

    table_clear(p4.SwitchEgress.diff_speed_function)
    table_clear(p4.SwitchEgress.actual_speed_function)
    table_clear(p4.SwitchEgress.target_speed_function)
    
def add_function_entry(table_char,speed,mask,value):
    """Add a function entry to a function table. 
    
    Args:
        table_char (str): the id of the function table
        speed: key value
        mask: mask of the ternary match
        value: function value in case of a match

    Valid table_char options:
        A: function on the actual speed
        T: function on the target speed
        D: function on the position difference
    """

    global p4

    def add_diff_speed_entry(speed,mask,value):
         p4.SwitchEgress.diff_speed_function.add_with_update_diff_speed(
            target_position=speed,
            target_position_mask=mask, 
            match_priority=None, 
            d=value, 
            pipe=None)
    
    def add_target_speed_entry(speed,mask,value):
         p4.SwitchEgress.target_speed_function.add_with_update_target_speed(
            target_speed=speed,
            target_speed_mask=mask, 
            match_priority=None, 
            d=value, 
            pipe=None)
    
    def add_actual_speed_entry(speed,mask,value):
         p4.SwitchEgress.actual_speed_function.add_with_update_actual_speed(
            speed=speed,
            speed_mask=mask, 
            match_priority=None, 
            d=value, 
            pipe=None)

    if table_char=="A":
        add_actual_speed_entry(speed,mask,value)
    if table_char=="T":
        add_target_speed_entry(speed,mask,value)
    if table_char=="D":
        add_diff_speed_entry(speed,mask,value)

def remove_range(robot_id,first,last):
    """Delete the trajectory points within the given range.

    Args:
        first (int): id of the first bunny to remove
        last (int): id of the last bunny to remove
    """

    for i in range(first,last+1):
        for j in range(6):
            try:
                p4.SwitchIngress.bunny.get(robot_id=robot_id,actual_bunny=i, jointid=j,print_ents=False).remove()
            except:
                pass
            try:
                p4.SwitchEgress.bunny_e.get(robot_id=robot_id,actual_bunny_id=i, jointid=j, print_ents=False).remove()
            except:
                pass

def get_actual_bunny_id(robot_id):
    """Return the value of the actual_bunny register."""

    global p4
    register = p4.SwitchIngress.r_actual_bunny
    register.operation_register_sync()
    value = register.get(robot_id).data[b'SwitchIngress.r_actual_bunny.f1']
    print(value)
    return value[0]

def set_actual_bunny_id(robot_id,bunny_id):
    """Set the value of the actual_bunny register."""

    global p4
    register = p4.SwitchIngress.r_actual_bunny
    register.mod(register_index=robot_id,f1=bunny_id)
    register.operation_register_sync()

def set_railway_switch(robot_id,from_id,to_id):
    """Set a redirection rule. (Insert an entry to the railwas_switch table.)
    
    If we use the bunny with from_id id then it changes the next_bunny_id to to_id.
    """

    global p4
    try:
        p4.SwitchIngress.railway_switch.add_with_change_next_bunny(
            robot_id=robot_id,
            actual_bunny=from_id,
            bunny_id=to_id,
            pipe=None)
    except Exception as e:
        print(e)

def unset_railway_switch(robot_id,from_id):
    """ Delete the redirection rule with the given source. (Remove an entry from the railway_switch table.)"""

    global p4
    try:
        p4.SwitchIngress.railway_switch.delete(robot_id,from_id)
    except Exception as e:
        print(e)

def run_tests():
    """Run some test measuring the required time to insert traj. points. """

    global time
    global clear_bunny_data
    global add_new_bunny
    global remove_range
    print("Rinning time measurements...")
    
    clear_bunny_data()
    print("TEST 1: inserting 100 bunny entry")
    ts = []
    for i in range(10):
        start = time.time()
        for k in range(100):
            add_new_bunny(i,i*1000+k,k,1000,[0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0])
        end = time.time()
        time_elapsed = end-start
        print("\tbatch "+str(i+1)+":\t"+str(time_elapsed))
        ts.append(time_elapsed)
    print("Avg: "+str(sum(ts)/len(ts)))
    
    clear_bunny_data()
    print("TEST 2: inserting 1000 bunny entry")
    ts = []
    for i in range(10):
        start = time.time()
        for k in range(1000):
            add_new_bunny(i,i*1000+k,k,1000,[0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0])
        end = time.time()
        time_elapsed = end-start
        print("\tbatch "+str(i+1)+":\t"+str(time_elapsed))
        ts.append(time_elapsed)
    print("Avg: "+str(sum(ts)/len(ts)))

    print("TEST 3: remove bunny range with 1000 entries")
    ts = []
    for i in range(10):
        start = time.time()
        remove_range(i,1000*i,1000*(i+1)-1)
        end = time.time()
        time_elapsed = end-start
        print("\tbatch "+str(i+1)+":\t"+str(time_elapsed))
        ts.append(time_elapsed)
    print("Avg: "+str(sum(ts)/len(ts)))

    clear_bunny_data()

# ***** MAIN *****

#clear_bunny_data()

#run_tests()

print("READY!")

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind(("localhost", 5555))
s.listen(1)

bunny_id_packer = struct.Struct("I")

while True:
    client_socket, client_addr = s.accept()
    print("Client connected: "+str(client_addr))

    cmd = unpack_int( recv_all(client_socket,4) )
    while cmd>0:
        if cmd==1:
            print("INFO: Receiving new bunny.")
            rid,bid,next_id,duration,pos,speeds = unpack_bunny_add_command(recv_all(client_socket,120))
            print("\t"+str(bid))
            add_new_bunny(rid,bid,next_id,duration,pos,speeds)
        elif cmd==2:
            print("INFO: Delete bunny range.")
            rid = unpack_int( recv_all(client_socket,4) )
            first = unpack_int( recv_all(client_socket,4) )
            last = unpack_int( recv_all(client_socket,4) )
            print("\t"+str(rid)+"["+str(first)+","+str(last)+"]")
            remove_range(rid,first,last)
        elif cmd==3:
            print("INFO: Clear every bunny.")
            clear_bunny_data()
        elif cmd==5:
            print("INFO: Dump bunny data.")
            dump_bunny_tables()
        elif cmd==7:
            print("INFO: clear functions")
            clear_function_data()
        elif cmd==8:
            print("INFO: add new function entry")
            c,speed,mask,value = unpack_function_add_command( recv_all(client_socket,32) )
            add_function_entry(c,speed,mask,value)
        elif cmd==9:
            print("INFO: get actual bunny id")
            rid = unpack_int( recv_all(client_socket,4) )
            bunny_id = get_actual_bunny_id(rid)
            client_socket.sendall(struct.Struct("I").pack(bunny_id))
        elif cmd==10:
            print("INFO: set railway switch")
            rid = unpack_int( recv_all(client_socket,4) )
            from_id = bunny_id_packer.unpack(recv_all(client_socket,4))[0]
            to_id = bunny_id_packer.unpack(recv_all(client_socket,4))[0]
            set_railway_switch(rid,from_id,to_id)
        elif cmd==11:
            print("INFO: unset railway switch")
            rid = unpack_int( recv_all(client_socket,4) )
            from_id = bunny_id_packer.unpack(recv_all(client_socket,4))[0]
            unset_railway_switch(rid,from_id)
        elif cmd==12:
            print("INFO: set actual bunny id")
            rid = unpack_int( recv_all(client_socket,4) )
            bunny_id = bunny_id_packer.unpack(recv_all(client_socket,4))[0]
            set_actual_bunny_id(rid,bunny_id)
            print("New actual bunny id: "+str(bunny_id))
        else:
            print("WARN: invalid command: "+str(cmd))
        cmd = unpack_int( recv_all(client_socket,4) )

    client_socket.close()
    print("BYE")
    if cmd==-2:
        print("STOP SERVER")
        break

s.close()

