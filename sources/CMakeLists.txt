 include_directories(
  ${PROJECT_SOURCE_DIR}/include
)
aux_source_directory(.  DIR_FUNCTIONS_SRCS)
add_library(functions ${DIR_FUNCTIONS_SRCS})
