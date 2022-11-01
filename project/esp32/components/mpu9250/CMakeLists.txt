
# include all header directories
set(INC_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../interface
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../example
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../test
    ${CMAKE_CURRENT_SOURCE_DIR}/interface
   )

# include all sources files
file(GLOB MPU9250_SRCS
     ${CMAKE_CURRENT_SOURCE_DIR}/../../../../src/*.c
    )

# include executable source
file(GLOB MPU9250_EXT_SRCS
     ${CMAKE_CURRENT_SOURCE_DIR}/../../../../example/*.c
     ${CMAKE_CURRENT_SOURCE_DIR}/../../../../test/*.c
     ${CMAKE_CURRENT_SOURCE_DIR}/interface/*.c
    )

set(src_files 
    ${MPU9250_EXT_SRCS}
    ${MPU9250_SRCS}
)

set(include_files 
    ${INC_DIRS}
)

idf_component_register(SRCS ${src_files}
                    INCLUDE_DIRS ${include_files}
                    REQUIRES driver
                    )