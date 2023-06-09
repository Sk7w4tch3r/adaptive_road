find_package(CGAL REQUIRED)
# include_directories(${CGAL_INCLUDE_DIRS})
# link_directories(${CGAL_LIBRARY_DIRS})
# add_definitions(${CGAL_DEFINITIONS})
# list(APPEND LIBRARIES ${CGAL_LIBRARIES})
# CGAL and its components
find_package( CGAL QUIET COMPONENTS  )

if ( NOT CGAL_FOUND )

  message(STATUS "This project requires the CGAL library, and will not be compiled.")
  return()  

endif()

# if(NOT CGAL_FOUND)
# 	message(WARNING "CGAL not found")
# endif()
