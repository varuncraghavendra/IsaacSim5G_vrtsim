# OAI notes

In a real deployment, replace `oai_udp_bridge.py` with the exact process that receives packets after OAI UE / user-plane termination.

The important contract is simple:

- teleop packets must reach UDP `16000`
- network metric packets must reach UDP `16001`

The broker does not care whether those packets came from:
- OAI RFSim
- OAI with namespaces
- a TAP/TUN pipeline
- a direct host-side test harness
