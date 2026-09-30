set(GIT_VERSION_SRC ${CMAKE_CURRENT_BINARY_DIR}/git_version.c)

# Add a custom command that produces git_version.c, plus
# a dummy output that's not actually produced, in order
# to force git_version.cmake to always be re-run before the build
add_custom_command(
    OUTPUT ${GIT_VERSION_SRC}
           ${CMAKE_CURRENT_BINARY_DIR}/_git_version.c
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_SOURCE_DIR}/cmake/git_version.cmake
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Updating git information..."
)

add_library(git_version STATIC ${GIT_VERSION_SRC})

target_include_directories(git_version PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
