#need fixing
# The ECVRP application can be executed by invoking the docker directly:
docker run --rm -v /ABSOLUTE_PATH_TO_ECVRP_APP:/ECVRP bapdock /ECVRP/src/run.jl /ECVRP/data/T/burma14.tsp -B 4 -Q 3 -u 3704

# toy instance (optimal is 3381)
docker run --rm -v /ABSOLUTE_PATH_TO_ECVRP_APP:/ECVRP bapdock /ECVRP/src/run.jl /ECVRP/data/toy.tsp -B 2 -Q 2 -D 2000 -u 3382

# Interactive mode:
docker run -it --rm -v /ABSOLUTE_PATH_TO_ECVRP_APP:/ECVRP bapdock

# Help with command line arguments
docker run --rm -v /ABSOLUTE_PATH_TO_ECVRP_APP:/ECVRP bapdock /ECVRP/src/run.jl --help

# The application directory (/ABSOLUTE_PATH_TO_ECVRP_APP) was mounted with -v as /ECVRP inside the container. Also, it is possible to mount a different directory to read/write solutions:
docker run --rm -v /ABSOLUTE_PATH_TO_ECVRP_APP:/ECVRP -v /ABSOLUTE_PATH_TO_OUTPUT:/OUT bapdock /ECVRP/src/run.jl /ECVRP/data/T/burma14.tsp -B 4 -Q 3 -u 3704 -o /OUT/burma14_B_4_Q_3.sol

# If you are calling docker through a bash terminal (e.g. Linux, MacOS or Docker QuickStart Terminal), you can call the script named VRPSolver in the demo directory. For example:
./VRPSolver data/T/burma14.tsp -B 4 -Q 3 -u 3704

# If you don't have permission to run VRPSolver script, call "chmod +x VRPSolver" before.
# This script must be called in the root directory of the application.

# Interactive mode:
./VRPSolver -it

# Help with command line arguments
./VRPSolver --help

# Running a batch of instances:
./VRPSolver -b testes10.batch

# Files with the extension .sh contain the call of VRPSolver for all instances individually.
