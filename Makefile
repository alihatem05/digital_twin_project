SERVER = vsitlmfabricvpc-nlb-ab1b44869757eb6a.elb.us-west-2.amazonaws.com
DOMAIN = AF_INET
KP     = 2.0
KI     = 0.1
KD     = 0.5
NOISE  = 0.0
PATH   = straight

sim:
	vsiSim LineFollowingRobot.dt

plant:
	cd $(shell pwd) && PYTHONPATH=. python3 src/plant/plant.py \
	  --domain=$(DOMAIN) --server-url=$(SERVER) \
	  --noise=$(NOISE) --path=$(PATH)

controller:
	cd $(shell pwd) && PYTHONPATH=. python3 src/controller/controller.py \
	  --domain=$(DOMAIN) --server-url=$(SERVER) \
	  --Kp=$(KP) --Ki=$(KI) --Kd=$(KD) --path=$(PATH)

visualizer:
	cd $(shell pwd) && PYTHONPATH=. python3 src/visualizer/visualizer.py \
	  --domain=$(DOMAIN) --server-url=$(SERVER) --label=Kp$(KP)_Ki$(KI)_Kd$(KD) --path=$(PATH)
