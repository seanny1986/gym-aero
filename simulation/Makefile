all: quadrotor_sim

quadrotor_sim: quadrotor_grt_rtw/quadrotor.c quadrotor_sim.c
	gcc -fpic -shared -o quadrotor_sim.so -Iquadrotor_grt_rtw -Isimulink_include -Iextern_include quadrotor_grt_rtw/quadrotor.c quadrotor_sim.c -lm
