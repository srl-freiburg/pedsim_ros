CXX = gcc -g -O0 -fno-inline

libpedsim.so: ped_obstacle.cpp ped_vector.cpp ped_waypoint.cpp ped_agent.cpp ped_scene.cpp ped_tree.cpp
	$(CXX) -fPIC -c ped_vector.cpp -o ped_vector.o
	$(CXX) -fPIC -c ped_scene.cpp -o ped_scene.o
	$(CXX) -fPIC -c ped_tree.cpp -o ped_tree.o
	$(CXX) -fPIC -c ped_obstacle.cpp -o ped_obstacle.o
	$(CXX) -fPIC -c ped_waypoint.cpp -o ped_waypoint.o
	$(CXX) -fPIC -c ped_agent.cpp -o ped_agent.o
	$(CXX) -shared  -Wl,-soname,libpedsim.so -o libpedsim.so ped_obstacle.o ped_vector.o ped_waypoint.o ped_agent.o ped_scene.o ped_tree.o

doc: doxygen.conf
	doxygen doxygen.conf

clean:
	rm *.o *.so