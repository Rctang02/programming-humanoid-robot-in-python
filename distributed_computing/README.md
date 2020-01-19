# 5: Distributed Computing
This week you need to implement distributed computing via [remote procedure call (RPC)](https://en.wikipedia.org/wiki/Remote_procedure_call), which has similiar interface you get with real robot via PyNaoQi.

The server and client have to be implemented together:
* [ServerAgent](./agent_server.py) provides remote RPC service;
* [ClientAgent](./agent_client.py) requests remote call from server and provides non-blocking capibility.


NOTE: this can't really run.The functions on the server will be called by the client, but there's no reaction showed in the simulation. Besides the keyframes folder AND the classifier have to be added into this folder so that the posture can be recognized and the keyframes can be executed (but can't see it because of the problem mentioned before). And also found that there's a small problem in the angle interpolation, maybe some keyframes run not so well.


