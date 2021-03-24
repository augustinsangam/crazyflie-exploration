if(NOT DEFINED EXPLORATION_DEP)
	cmake_minimum_required(VERSION 3.16)
	project(exploration VERSION 0.1.0 LANGUAGES CXX)

	set(CMAKE_CXX_STANDARD 17)
	set(CMAKE_CXX_STANDARD_REQUIRED ON)
	set(CMAKE_CXX_EXTENSIONS OFF)

	set(EXPLORATION_METHOD "1")
endif()

if(NOT DEFINED EXPLORATION_METHOD)
	message(FATAL_ERROR "Please define EXPLORATION_METHOD")
endif()

add_library(exploration STATIC)
target_include_directories(exploration PUBLIC "./include")
target_compile_definitions(exploration PUBLIC
	EXPLORATION_METHOD=${EXPLORATION_METHOD})
target_sources(exploration PRIVATE
	"./src/exploration/SGBA.cpp"
	"./src/exploration/StateMachine.cpp"
	"./src/exploration/WallFollowing.cpp"
	"./src/exploration/WallFollowingWithAvoid.cpp"
	"./src/exploration/median_filter.cpp")
