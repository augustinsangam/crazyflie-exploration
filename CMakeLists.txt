if(NOT DEFINED EXPLORATION_DEP)
	cmake_minimum_required(VERSION 3.16)
	project(exploration VERSION 0.1.0 LANGUAGES CXX)

	set(CMAKE_CXX_STANDARD 17)
	set(CMAKE_CXX_STANDARD_REQUIRED ON)
	set(CMAKE_CXX_EXTENSIONS OFF)
endif()

add_library(exploration STATIC)
target_include_directories(exploration PUBLIC ./include)
target_compile_definitions(exploration PUBLIC
	EXPLORATION_METHOD=1
	TICKS_PER_FSM_LOOP=10)
target_sources(exploration PRIVATE
	"./src/sgba.cpp"
	"./src/state_machine.cpp"
	"./src/wall_following.cpp"
	"./src/wall_following_with_avoid.cpp")
