set(includes "."
             "../utils")

idf_component_register(SRCS "bell103-host.cpp" "cmx865a.cpp" "CircularByteBuffer.cpp"
                    INCLUDE_DIRS ${includes})
