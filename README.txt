To Run:
1. Start Redis:
	redis-cli
	if any problem, run: brew services start redis
2. In redis-cli:
	flushdb
3. ./sawyer sawyer.urdf
4. python test.py

Code Structure:
Set a desired position and orientation in test.py, test.py will read q and dq, send them to sawyer(cpp) through redis. sawyer(cpp) reads q and dq, calculate the torque needed at the moment, and send to test.py through redis. teset.py then apply the torque received and update q and dq.



