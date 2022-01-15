set(component_srcs "src/bm_wifi.c")

idf_component_register(SRCS "${component_srcs}"
                        INCLUDE_DIRS "include"
                        PRIV_INCLUDE_DIRS ""
                        PRIV_REQUIRES ""
                        REQUIRES "")
