############################################################################

px4_add_library(IMBControl
	IMBControl.cpp
)
target_include_directories(IMBControl
	PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}
)
