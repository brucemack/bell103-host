set(includes "."
             "../utils")

idf_component_register(SRCS "bell103-host.cpp"
                    INCLUDE_DIRS ${includes})
