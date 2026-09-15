cmake_minimum_required(VERSION 3.22)
project(mcl_standalone)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

########  COPY THIS PART IN EACH CMakeLists.txt ########
set(SEARCH_DIR "${CMAKE_CURRENT_LIST_DIR}")

while(TRUE)
    if(EXISTS "${SEARCH_DIR}/env_loader.cpp"
       AND EXISTS "${SEARCH_DIR}/.env")

        set(ROOT_DIR "${SEARCH_DIR}")
        break()
    endif()

    get_filename_component(PARENT_DIR "${SEARCH_DIR}" DIRECTORY)

    if(PARENT_DIR STREQUAL SEARCH_DIR)
        message(FATAL_ERROR "Could not find env_loader.cpp and .env")
    endif()

    set(SEARCH_DIR "${PARENT_DIR}")
endwhile()
########################################################

include_directories(include)

add_executable(sir src/SIR.cpp "${ROOT_DIR}/env_loader.cpp")
add_executable(map src/Map.cpp "${ROOT_DIR}/env_loader.cpp")
