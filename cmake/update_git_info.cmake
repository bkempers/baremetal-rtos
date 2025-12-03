find_program(GIT_EXECUTABLE git)

if(GIT_EXECUTABLE)
    execute_process(
        COMMAND ${GIT_EXECUTABLE} rev-parse --abbrev-ref HEAD
        WORKING_DIRECTORY ${SOURCE_DIR}
        OUTPUT_VARIABLE GIT_BRANCH
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
    
    execute_process(
        COMMAND ${GIT_EXECUTABLE} rev-parse HEAD
        WORKING_DIRECTORY ${SOURCE_DIR}
        OUTPUT_VARIABLE GIT_COMMIT_HASH
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
    
    execute_process(
        COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
        WORKING_DIRECTORY ${SOURCE_DIR}
        OUTPUT_VARIABLE GIT_COMMIT_SHORT
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
    
    execute_process(
        COMMAND ${GIT_EXECUTABLE} log -1 --format=%cd --date=iso-strict
        WORKING_DIRECTORY ${SOURCE_DIR}
        OUTPUT_VARIABLE GIT_COMMIT_DATE
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
    
    execute_process(
        COMMAND ${GIT_EXECUTABLE} diff-index --quiet HEAD --
        WORKING_DIRECTORY ${SOURCE_DIR}
        RESULT_VARIABLE GIT_DIRTY_RESULT
        ERROR_QUIET
    )
    
    if(GIT_DIRTY_RESULT EQUAL 0)
        set(GIT_DIRTY "clean")
    else()
        set(GIT_DIRTY "dirty")
    endif()
else()
    set(GIT_BRANCH "unknown")
    set(GIT_COMMIT_HASH "unknown")
    set(GIT_COMMIT_SHORT "unknown")
    set(GIT_COMMIT_DATE "unknown")
    set(GIT_DIRTY "unknown")
endif()

string(TIMESTAMP BUILD_TIMESTAMP "%Y-%m-%d %H:%M:%S" UTC)

configure_file(
    ${SOURCE_DIR}/support/git_info.h.in
    ${BINARY_DIR}/Application/inc/git_info.h
    @ONLY
)
