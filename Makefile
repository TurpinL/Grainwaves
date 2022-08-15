# Project Name
TARGET = Grainwaves

# Sources
CPP_SOURCES = Grainwaves.cpp Daisy_SSD1327/Daisy_SSD1327.cpp

# Library Locations
LIBDAISY_DIR = ../../libDaisy/
DAISYSP_DIR = ../../DaisySP/

# Core location, and generic Makefile.
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile
