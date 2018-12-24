thirdparty_path_ = $(CURDIR)/thirdparty
  
## [build]
#build_cflags = -O0 -g -UNDEBUG
#build_cflags = -O3 -DNDEBUG
build_cflags = 
build_ldflags =
build_ldlibs =

## [Boost]
boost_home_ = $(HOME)/sys/src/git/boost
#boost_home_ = $(thirdparty_path_)/boost_1_65_0
boost_cflags = -I$(boost_home_)
boost_ldflags = -L$(boost_home_)/stage/lib
boost_ldlibs =

## [CPLEX]
cplex_home_ = $(HOME)/sys/opt/optim/ibm/ILOG/CPLEX_Studio1271
cplex_cflags = -I$(cplex_home_)/cplex/include -I$(cplex_home_)/cpoptimizer/include -I$(cplex_home_)/concert/include -DIL_STD
cplex_cflags += -DDCS_FOG_VM_ALLOC_USE_NATIVE_CP_SOLVER
cplex_ldflags = -L$(cplex_home_)/cplex/lib/x86-64_linux/static_pic -L$(cplex_home_)/cpoptimizer/lib/x86-64_linux/static_pic -L$(cplex_home_)/concert/lib/x86-64_linux/static_pic
cplex_ldlibs = -lilocplex -lcp -lcplex -lconcert -lm -lpthread

## [dcsxx-commons]
# - Path to dcsxx-commons project
dcs_commons_home_ = $(HOME)/Projects/src/dcsxx-commons
dcs_commons_cflags = -I$(dcs_commons_home_)/inc
dcs_commons_ldflags =
dcs_commons_ldlibs =

## [YAML-cpp]
yamlcpp_cflags =
yamlcpp_ldflags =
yamlcpp_ldlibs = -lyaml-cpp
