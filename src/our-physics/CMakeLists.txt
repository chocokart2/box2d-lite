project(samples LANGUAGES CXX)

set(OUR_PHYSICS_SOURCE_FILES
	OurArbiter.cpp
	OurBody.cpp
	OurCollide.cpp
	OurJoint.cpp
	OurWorld.cpp)

set(OUR_PHYSICS_HEADER_FILES
	../include/our-physics/OurArbiter.h
	../include/our-physics/OurBody.h
	../include/our-physics/OurJoint.h
	../include/our-physics/OurMath.h
	../include/our-physics/OurWorld.h)

add_library(our-phycics STATIC ${OUR_PHYSICS_SOURCE_FILES} ${OUR_PHYSICS_HEADER_FILES})
target_include_directories(our-phycics PUBLIC ../include/our-phycics)