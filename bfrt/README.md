# Control plane

## Main files:

- run.sh: start the control plane bfrt code
- setup.py: bfrt control plane
- client.py: debug/config CLI
- proxy.py: adapter/proxy between the control plane and the ROS 
- mock_proxy_client.py: simplified client to the proxy.py mocking the ROS part
- trajs.csv: example trajectory

## setup.py

The bfrt control plane code receives TCP connections on port 5555.

Each request is introduced with a command id. Read the client.py for more details.

You can not start this script directly. Use the run.sh.

## proxy.py

This proxy receives TCP connections on port 10000.

Each request has the following structure:
- command type (uint32_t in network byte order): command type id
- traj. length (uint32_t in network byte order): length of the csv traj in bytes
- csv traj. (bytes): trajectory to upload in csv format

The proxy sends an ACK message after completing the request. The ack message is a simple 1 (uint32_t in network byte order).

Valid command types:
- 0: Upload traj. in reset mode. It deletes every traj. points in the switch before uploading this one. 
- 1: Upload traj. in append mode. Concatenate this traj to the end of the previous one.

The proxy opens a new connection to the setup.py for each request for debugging purposes only.

